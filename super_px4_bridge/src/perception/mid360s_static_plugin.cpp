// Static S2 ray/clock probe. MID360 vendor trace + MID360S FOV is an approximation.
// No actuation, motion compensation, calibrated optics, intensity or noise claims.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <zlib.h>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace gazebo {
namespace {
ros::Time Stamp(uint64_t ns) { return ros::Time(ns/1000000000ULL, ns%1000000000ULL); }
template<class T> void Put(std::vector<uint8_t> &data, size_t p, T value) {
  std::memcpy(data.data()+p, &value, sizeof(value));
}
void Field(sensor_msgs::PointCloud2 &m, const std::string &name, uint32_t offset, uint8_t type) {
  sensor_msgs::PointField f; f.name=name; f.offset=offset; f.datatype=type; f.count=1;
  m.fields.push_back(f);
}
}

class Mid360sStaticPlugin : public WorldPlugin {
 public:
  ~Mid360sStaticPlugin() override { update_.reset(); node_.reset(); }
  void Load(physics::WorldPtr world, sdf::ElementPtr s) override {
    try {
      world_=world;
      if (!ros::isInitialized()) throw std::runtime_error("Load gazebo_ros_api_plugin first");
      if (world_->Physics()->GetType()!="ode") throw std::runtime_error("S2 requires ODE");
      auto model=world_->ModelByName("s2_vehicle");
      sensor_=model ? model->GetLink("lidar_frame") : nullptr;
      if (!sensor_ || !model->IsStatic()) throw std::runtime_error("Missing STATIC s2_vehicle/lidar_frame");
      for (const auto &m:world_->Models())
        if (!m->IsStatic()) throw std::runtime_error("S2 world must contain only static models");
      pose_=sensor_->WorldPose();
      rate_=s->Get<unsigned int>("point_rate"); frame_rate_=s->Get<unsigned int>("frame_rate");
      imu_rate_=s->Get<unsigned int>("imu_rate");
      min_=s->Get<double>("min_range"); max_=s->Get<double>("max_range");
      low_=s->Get<double>("low_deg"); high_=s->Get<double>("high_deg");
      sigma_=s->Get<double>("noise_sigma");
      if (!rate_ || !frame_rate_ || !imu_rate_ || rate_>1000000 || rate_%frame_rate_ ||
          1000000000ULL%rate_ || 1000000000ULL%imu_rate_ || !std::isfinite(sigma_) || sigma_<0 ||
          !(0<min_ && min_<max_) || !(-90<low_ && low_<high_ && high_<90))
        throw std::runtime_error("Invalid scan parameters");
      point_ns_=1000000000ULL/rate_; imu_ns_=1000000000ULL/imu_rate_;
      frame_points_=rate_/frame_rate_; rng_.seed(s->Get<unsigned int>("seed"));
      gzFile file=gzopen(s->Get<std::string>("trace_file").c_str(), "rb");
      if (!file) throw std::runtime_error("Cannot open trace");
      char line[256]; size_t row=0;
      while (gzgets(file,line,sizeof(line))) {
        if (row++==0) continue;
        double sample, azimuth, zenith;
        if (std::sscanf(line,"%lf,%lf,%lf",&sample,&azimuth,&zenith)!=3 ||
            !std::isfinite(sample) || !std::isfinite(azimuth) || !std::isfinite(zenith)) {
          gzclose(file); throw std::runtime_error("Invalid trace row");
        }
        // Vendor code rotates pitch=(zenith-90deg), hence elevation=90-zenith.
        angles_.push_back({azimuth*M_PI/180., (90.-zenith)*M_PI/180.});
      }
      gzclose(file);
      if (angles_.empty()) throw std::runtime_error("Empty trace");
      ray_=boost::dynamic_pointer_cast<physics::RayShape>(world_->Physics()->CreateShape("ray",physics::CollisionPtr()));
      if (!ray_) throw std::runtime_error("Cannot create ODE ray");
      node_.reset(new ros::NodeHandle);
      points_pub_=node_->advertise<sensor_msgs::PointCloud2>(s->Get<std::string>("points_topic"),10);
      truth_pub_=node_->advertise<sensor_msgs::PointCloud2>(s->Get<std::string>("truth_topic"),10);
      imu_pub_=node_->advertise<sensor_msgs::Imu>(s->Get<std::string>("imu_topic"),100);
      update_=event::Events::ConnectWorldUpdateBegin(std::bind(&Mid360sStaticPlugin::Update,this,std::placeholders::_1));
      gzmsg << "S2 static trace ready: " << angles_.size() << " angles, " << rate_ << " rays/s\n";
    } catch (const std::exception &e) { gzerr << "MID360S_S2_LOAD_FAILED: " << e.what() << "\n"; }
  }
 private:
  struct Angle { double az, el; };
  void NewFrame() {
    cloud_=sensor_msgs::PointCloud2(); truth_=sensor_msgs::PointCloud2();
    for (auto *m:{&cloud_,&truth_}) {
      m->header.seq=frame_; m->header.stamp=Stamp(epoch_+frame_*frame_points_*point_ns_);
      m->header.frame_id="mid360s_lidar"; m->height=1; m->is_bigendian=false; m->is_dense=true;
      m->point_step=32; m->data.reserve(frame_points_*32);
      Field(*m,"x",0,7); Field(*m,"y",4,7); Field(*m,"z",8,7);
    }
    // Same first seven fields/offsets as official driver2 PointXYZRTLT; offset_time is extra.
    Field(cloud_,"intensity",12,7); Field(cloud_,"tag",16,2); Field(cloud_,"line",17,2);
    Field(cloud_,"timestamp",18,8); Field(cloud_,"offset_time",26,6);
    // Clean intersection and semantic labels are ONLY on this separate scoring topic.
    Field(truth_,"label",12,6); Field(truth_,"sample_index",16,6);
    Field(truth_,"timestamp",20,8); Field(truth_,"offset_time",28,6);
  }
  uint32_t Label(const std::string &entity) const {
    if (entity.find("s2_target::")!=std::string::npos) {
      if (entity.find("::body")!=std::string::npos) return 1;
      if (entity.find("::key")!=std::string::npos) return 2;
      if (entity.find("::handle")!=std::string::npos) return 3;
    }
    if (entity.find("s2_support::")!=std::string::npos) return 4;
    if (entity.find("s2_vehicle::")!=std::string::npos) return 6;
    return 5;
  }
  void Sample(uint64_t ns) {
    const auto &a=angles_[index_%angles_.size()];
    double degrees=a.el*180./M_PI;
    if (degrees<low_ || degrees>high_) return;  // Drop out-of-FOV rays, never remap/oversample target.
    ignition::math::Vector3d local(std::cos(a.el)*std::cos(a.az), std::cos(a.el)*std::sin(a.az), std::sin(a.el));
    auto direction=pose_.Rot().RotateVector(local);
    ray_->SetPoints(pose_.Pos(),pose_.Pos()+direction*max_);
    double range=max_; std::string entity;
    ray_->GetIntersection(range,entity);
    if (entity.empty() || !std::isfinite(range) || range<min_ || range>=max_) return;
    const double noisy=range+sigma_*normal_(rng_);
    if (noisy<min_ || noisy>=max_) return;
    auto p=local*noisy, clean=local*range;
    const size_t pos=cloud_.data.size(); cloud_.data.resize(pos+32,0); truth_.data.resize(pos+32,0);
    for (int axis=0;axis<3;++axis) {
      Put<float>(cloud_.data,pos+axis*4,static_cast<float>(p[axis]));
      Put<float>(truth_.data,pos+axis*4,static_cast<float>(clean[axis]));
    }
    Put<float>(cloud_.data,pos+12,100.f); // Synthetic constant, no reflectivity model.
    cloud_.data[pos+17]=index_%4; // Synthetic channel index, not validated physical MID360S lines.
    const uint32_t offset=(index_%frame_points_)*point_ns_;
    // driver2 PointXYZRTLT.timestamp is absolute nanoseconds, NOT seconds.
    Put<double>(cloud_.data,pos+18,static_cast<double>(ns)); Put<uint32_t>(cloud_.data,pos+26,offset);
    Put<uint32_t>(truth_.data,pos+12,Label(entity)); Put<uint32_t>(truth_.data,pos+16,index_%angles_.size());
    Put<double>(truth_.data,pos+20,static_cast<double>(ns)); Put<uint32_t>(truth_.data,pos+28,offset);
  }
  void Update(const common::UpdateInfo &info) {
    const uint64_t now=static_cast<uint64_t>(info.simTime.sec)*1000000000ULL+info.simTime.nsec;
    if (!started_) { epoch_=now; next_imu_=epoch_; started_=true; NewFrame(); }
    if (now<last_now_ || sensor_->WorldPose().Pos().Distance(pose_.Pos())>1e-9 ||
        (pose_.Rot().Inverse()*sensor_->WorldPose().Rot()).Euler().Length()>1e-9) {
      gzerr << "S2_STATIC_ONLY: time reset or sensor movement, stopping publication\n";
      update_.reset(); return;
    }
    last_now_=now;
    while (next_imu_<=now) {
      sensor_msgs::Imu imu; imu.header.seq=imu_seq_++; imu.header.stamp=Stamp(next_imu_);
      imu.header.frame_id="mid360s_lidar"; imu.orientation_covariance[0]=-1;
      auto force=pose_.Rot().RotateVectorReverse(-world_->Gravity());
      imu.linear_acceleration.x=force.X(); imu.linear_acceleration.y=force.Y(); imu.linear_acceleration.z=force.Z();
      imu_pub_.publish(imu); next_imu_+=imu_ns_;
    }
    while (epoch_+index_*point_ns_<=now) {
      Sample(epoch_+index_*point_ns_); ++index_;
      if (index_%frame_points_==0) {
        for (auto *m:{&cloud_,&truth_}) { m->width=m->data.size()/32; m->row_step=m->width*32; }
        points_pub_.publish(cloud_); truth_pub_.publish(truth_); ++frame_; NewFrame();
      }
    }
  }
  physics::WorldPtr world_; physics::LinkPtr sensor_; physics::RayShapePtr ray_;
  ignition::math::Pose3d pose_; event::ConnectionPtr update_;
  std::unique_ptr<ros::NodeHandle> node_; ros::Publisher points_pub_,truth_pub_,imu_pub_;
  std::vector<Angle> angles_; std::mt19937 rng_; std::normal_distribution<double> normal_{0.,1.};
  sensor_msgs::PointCloud2 cloud_,truth_;
  uint64_t epoch_=0,index_=0,next_imu_=0,point_ns_=0,imu_ns_=0,last_now_=0;
  unsigned int rate_=0,frame_rate_=0,imu_rate_=0,frame_points_=0,frame_=0,imu_seq_=0;
  double min_=0,max_=0,low_=0,high_=0,sigma_=0; bool started_=false;
};
GZ_REGISTER_WORLD_PLUGIN(Mid360sStaticPlugin)
}  // namespace gazebo
