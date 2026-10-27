"""Pure NumPy static known-shape estimator. Input: XYZ, local shape and static IMU up.

No bag/scene readers, target world pose, semantic labels, control clients or publishers.
"""
import itertools
import math
import time
import numpy as np


def validate_config(cfg):
    if cfg['schema_version']!=1 or cfg['frame']!='mid360s_lidar': raise ValueError('Unknown S3 schema/frame')
    low=np.asarray(cfg['roi_min_m'],dtype=float); high=np.asarray(cfg['roi_max_m'],dtype=float)
    if low.shape!=(3,) or high.shape!=(3,) or not np.isfinite([*low,*high]).all() or np.any(low>=high):
        raise ValueError('Invalid coarse ROI')
    for key in ('voxel_m','plane_distance_m','plane_min_extent_m','plane_connectivity_m','cluster_radius_m',
            'cluster_max_diagonal_m','model_sample_m','huber_m','inlier_distance_m','max_fit_rms_m',
            'max_key_reverse_rms_m','ambiguity_score_gap_m','different_pose_angle_deg','different_pose_translation_m'):
        if not math.isfinite(cfg[key]) or cfg[key]<=0: raise ValueError('Invalid '+key)
    for key in ('plane_min_points','plane_iterations','plane_rounds','cluster_min_points','cluster_max_points',
                'optimizer_iterations','min_key_points','yaw_seed_step_deg'):
        if type(cfg[key]) is not int or cfg[key]<=0: raise ValueError('Invalid '+key)
    if (cfg['cluster_min_points']>cfg['cluster_max_points'] or not 0<cfg['min_inlier_fraction']<=1 or
            not 0<cfg['upright_tilt_limit_deg']<90 or 360%cfg['yaw_seed_step_deg']):
        raise ValueError('Invalid limits or yaw sampling')


def skew(v):
    x,y,z=v
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def exp(v):
    angle=float(np.linalg.norm(v)); K=skew(v)
    return np.eye(3)+K if angle<1e-10 else np.eye(3)+math.sin(angle)/angle*K+(1-math.cos(angle))/angle**2*K@K


def rotation_deg(rpy):
    r,p,y=np.deg2rad(rpy); cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
        [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


def shape_from_geometry(config):
    """Whitelist LOCAL dimensions only: absolute target pose/arm/observation inputs are ignored."""
    parts=[]
    for p in config['target']['parts']:
        size=np.array(p['size_m'],dtype=float); pose=np.array(p['xyz_rpy_deg'],dtype=float)
        if size.shape!=(3,) or pose.shape!=(6,) or not np.isfinite([*size,*pose]).all() or np.any(size<=0):
            raise ValueError('Invalid local shape')
        parts.append(dict(name=p['name'],half=size/2,center=pose[:3],R=rotation_deg(pose[3:])))
    if {p['name'] for p in parts}!={'body','key','handle'} or len(parts)!=3:
        raise ValueError('First S3 profile requires body/key/handle')
    return parts


def voxel(points, size):
    if not len(points): return points.copy()
    key=np.floor(points/size).astype(np.int64)
    _,index=np.unique(key,axis=0,return_index=True)
    return points[np.sort(index)]


def large_plane_patch_mask(points,cfg):
    """Only CONNECTED large patches may be removed, not distant coplanar fragments."""
    keys,inverse_key=np.unique(np.floor(points/cfg['plane_connectivity_m']).astype(int),axis=0,return_inverse=True)
    lookup={tuple(k):i for i,k in enumerate(keys)}; component=np.full(len(keys),-1,dtype=int)
    shifts=list(itertools.product((-1,0,1),repeat=3)); number=0; accepted=[]
    for seed in range(len(keys)):
        if component[seed]>=0: continue
        component[seed]=number; queue=[seed]
        for i in queue:
            for shift in shifts:
                j=lookup.get(tuple(keys[i]+shift))
                if j is not None and component[j]<0: component[j]=number; queue.append(j)
        mask=component[inverse_key]==number; p=points[mask]
        if len(p)>=cfg['plane_min_points'] and np.linalg.norm(p.max(0)-p.min(0))>=cfg['plane_min_extent_m']:
            accepted.append(number)
        number+=1
    return np.isin(component[inverse_key],accepted)


def remove_planes(points,cfg):
    p=points.copy(); rng=np.random.default_rng(cfg['seed']); records=[]
    for _ in range(cfg['plane_rounds']):
        if len(p)<cfg['plane_min_points']: break
        triples=p[rng.integers(0,len(p),(cfg['plane_iterations'],3))]
        n=np.cross(triples[:,1]-triples[:,0],triples[:,2]-triples[:,0]); length=np.linalg.norm(n,axis=1)
        valid=length>1e-8; n=n[valid]/length[valid,None]
        if len(n)==0: break
        d=np.einsum('ij,ij->i',n,triples[valid,0])
        distance=abs(p@n.T-d); counts=np.sum(distance<cfg['plane_distance_m'],axis=0)
        order=np.argsort(-counts); chosen=None
        for i in order[:12]:
            mask=distance[:,i]<cfg['plane_distance_m']; q=p[mask]
            if len(q)<cfg['plane_min_points']: continue
            # Never remove an object-sized face merely because it is planar.
            if np.linalg.norm(q.max(0)-q.min(0))<cfg['plane_min_extent_m']: continue
            connected=large_plane_patch_mask(q,cfg)
            if not np.any(connected): continue
            chosen=np.zeros(len(p),dtype=bool); chosen[np.flatnonzero(mask)[connected]]=True
            records.append(dict(points=int(np.sum(chosen)),normal=n[i].tolist(),offset_m=float(d[i])))
            break
        if chosen is None: break
        p=p[~chosen]
    return p,records


def clusters(points, radius):
    keys=np.floor(points/radius).astype(int); grid={}
    for i,k in enumerate(map(tuple,keys)): grid.setdefault(k,[]).append(i)
    seen=np.zeros(len(points),dtype=bool); groups=[]; neighbors=list(itertools.product((-1,0,1),repeat=3))
    for seed in range(len(points)):
        if seen[seed]: continue
        seen[seed]=True; queue=[seed]
        for i in queue:
            for shift in neighbors:
                indices=grid.get(tuple(keys[i]+shift),[])
                candidates=np.array([j for j in indices if not seen[j]],dtype=int)
                if not len(candidates): continue
                selected=candidates[np.sum((points[candidates]-points[i])**2,axis=1)<=radius**2]
                seen[selected]=True; queue.extend(selected.tolist())
        groups.append(points[np.array(queue)])
    return sorted(groups,key=len,reverse=True)


def distances(local, parts):
    values=[]
    for part in parts:
        q=abs((local-part['center'])@part['R'])-part['half']
        values.append(np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(np.max(q,axis=1),0))
    return np.array(values).T


def residual(points,R,t,parts):
    # Union signed distance; interior shared joins are irrelevant to observed outer surfaces.
    return np.min(distances((points-t)@R,parts),axis=1)


def huber(r, scale):
    a=abs(r)
    return np.where(a<=scale,r*r,2*scale*a-scale**2)


def refine(points,R,t,parts,cfg):
    R=R.copy(); t=t.copy(); last=math.inf
    for _ in range(cfg['optimizer_iterations']):
        r=residual(points,R,t,parts); value=float(np.mean(huber(r,cfg['huber_m'])))
        if last-value<1e-11: break
        last=value; J=np.empty((len(points),6))
        for a in range(6):
            v=np.zeros(3); v[a%3]=1e-5 if a<3 else 1e-4
            J[:,a]=(residual(points,R,t+v,parts)-r)/1e-5 if a<3 else (residual(points,R@exp(v),t,parts)-r)/(1e-4*.1)
        w=np.sqrt(np.minimum(1,cfg['huber_m']/np.maximum(abs(r),1e-12)))
        step=np.linalg.lstsq(np.vstack([J*w[:,None],np.eye(6)*.003]),np.r_[-r*w,np.zeros(6)],rcond=None)[0]
        dt=step[:3]; dr=step[3:]/.1
        dt*=min(1,.02/max(1e-12,np.linalg.norm(dt))); dr*=min(1,.2/max(1e-12,np.linalg.norm(dr)))
        improved=False
        for factor in (1.,.5,.25,.125):
            nextR=R@exp(dr*factor); nextt=t+dt*factor
            if np.mean(huber(residual(points,nextR,nextt,parts),cfg['huber_m']))<value:
                R,t=nextR,nextt; improved=True; break
        if not improved: break
    return R,t


def model_samples(parts, spacing):
    pts=[]; normal=[]; ids=[]
    for index,part in enumerate(parts):
        for axis,sign in itertools.product(range(3),(-1.,1.)):
            other=[i for i in range(3) if i!=axis]; n=np.maximum(2,np.ceil(part['half'][other]*2/spacing).astype(int))
            for i,j in itertools.product(range(n[0]),range(n[1])):
                p=np.zeros(3); p[axis]=part['half'][axis]*sign
                p[other]=-part['half'][other]+(np.array([i,j])+.5)*part['half'][other]*2/n
                q=part['R']@p+part['center']
                # Discard model surface patches enclosed inside another part.
                if any(np.all(abs((q-b['center'])@b['R'])<b['half']-1e-7) for b in parts if b is not part): continue
                pts.append(q); normal.append(part['R'][:,axis]*sign); ids.append(index)
    return np.asarray(pts),np.asarray(normal),np.asarray(ids)


def nearest_dist(a,b):
    if not len(a) or not len(b): return np.full(len(a),np.inf)
    result=[]
    for start in range(0,len(a),256):
        d=np.sum((a[start:start+256,None,:]-b[None,:,:])**2,axis=2)
        result.extend(np.sqrt(np.min(d,axis=1)))
    return np.array(result)


def rotation_angle(R):
    return math.degrees(math.acos(float(np.clip((np.trace(R)-1)/2,-1,1))))


def score(points,R,t,parts,sampled,cfg):
    local=(points-t)@R; dist=distances(local,parts); r=np.min(dist,axis=1)
    model,normal,ids=sampled; sensor=-R.T@t
    visible=np.sum((sensor-model)*normal,axis=1)>0
    # Only local shape self-occlusion. No scene/target truth is available here.
    from check_installation import blocked
    for index,part in enumerate(parts):
        T=np.eye(4); T[:3,:3]=part['R']; T[:3,3]=part['center']
        visible &= ~((ids!=index)&blocked(sensor,model,T,part['half']))
    reverse=nearest_dist(model[visible],local); vis_ids=ids[visible]
    key=next(i for i,p in enumerate(parts) if p['name']=='key')
    assigned=np.argmin(abs(dist),axis=1)
    key_points=int(np.sum((assigned==key)&(abs(dist[:,key])<=cfg['inlier_distance_m'])))
    key_reverse=reverse[vis_ids==key]
    rms=float(np.sqrt(np.mean(r*r)))
    reverse_rms=float(np.sqrt(np.mean(reverse*reverse))) if len(reverse) else 1.
    key_rms=float(np.sqrt(np.mean(key_reverse*key_reverse))) if len(key_reverse) else 1.
    return dict(score_m=.45*rms+.25*reverse_rms+.30*key_rms,fit_rms_m=rms,
        reverse_rms_m=reverse_rms,key_reverse_rms_m=key_rms,key_points=key_points,
        inlier_fraction=float(np.mean(abs(r)<=cfg['inlier_distance_m'])),
        R=R.tolist(),position_lidar_m=t.tolist())


def fit_cluster(points,parts,cfg,up):
    sampled=model_samples(parts,cfg['model_sample_m'])
    center=np.mean(points,axis=0)
    _,axes=np.linalg.eigh(np.cov((points-center).T))
    model_center=(sampled[0].min(0)+sampled[0].max(0))/2
    fits=[]; seeds=[]
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((-1,1),repeat=3):
            R=axes[:,perm]*signs
            if np.linalg.det(R)>0 and R[:,2]@up>=math.cos(math.radians(cfg['upright_tilt_limit_deg'])):
                seeds.append(R)
    forward=np.array([1.,0,0]) if abs(up[0])<.9 else np.array([0.,1.,0])
    forward-=up*(forward@up); forward/=np.linalg.norm(forward)
    base=np.column_stack([forward,np.cross(up,forward),up])
    for yaw in range(0,360,cfg['yaw_seed_step_deg']): seeds.append(base@rotation_deg([0,0,yaw]))
    for R in seeds:
        t=center-R@model_center
        solvedR,solvedt=refine(points,R,t,parts,cfg)
        if solvedR[:,2]@up<math.cos(math.radians(cfg['upright_tilt_limit_deg'])): continue
        fits.append(score(points,solvedR,solvedt,parts,sampled,cfg))
    return sorted(fits,key=lambda r:r['score_m'])


def estimate(points,parts,cfg,up_lidar=(0.,0.,1.)):
    validate_config(cfg)
    begin=time.perf_counter(); p=np.asarray(points,dtype=float)
    if p.ndim!=2 or p.shape[1]!=3 or not np.isfinite(p).all(): raise ValueError('Nonfinite or malformed XYZ')
    up=np.asarray(up_lidar,dtype=float)
    if up.shape!=(3,) or not np.isfinite(up).all() or abs(np.linalg.norm(up)-1)>.01:
        raise ValueError('Invalid measured up direction')
    up/=np.linalg.norm(up)
    roi=np.all((p>=cfg['roi_min_m'])&(p<=cfg['roi_max_m']),axis=1)
    p=voxel(p[roi],cfg['voxel_m']); remaining,planes=remove_planes(p,cfg)
    groups=clusters(remaining,cfg['cluster_radius_m'])
    candidates=[]; group_report=[]
    for index,g in enumerate(groups):
        extent=g.max(0)-g.min(0); usable=(cfg['cluster_min_points']<=len(g)<=cfg['cluster_max_points'] and
            np.linalg.norm(extent)<=cfg['cluster_max_diagonal_m'])
        group_report.append(dict(index=index,points=len(g),extent_m=extent.tolist(),candidate=bool(usable)))
        if not usable: continue
        fits=fit_cluster(g,parts,cfg,up)
        for f in fits: f.update(cluster=index,cluster_points=len(g)); candidates.append(f)
    candidates.sort(key=lambda r:r['score_m']); reasons=[]
    if not candidates: reasons=['no_geometric_candidate']; best=None
    else:
        best=candidates[0]; R=np.array(best['R']); t=np.array(best['position_lidar_m'])
        alternate=next((f for f in candidates[1:] if f['cluster']!=best['cluster'] or
            rotation_angle(R.T@np.array(f['R']))>cfg['different_pose_angle_deg'] or
            np.linalg.norm(t-f['position_lidar_m'])>cfg['different_pose_translation_m']),None)
        gap=alternate['score_m']-best['score_m'] if alternate else None
        best['distinct_alternative_score_gap_m']=gap
        if best['fit_rms_m']>cfg['max_fit_rms_m']: reasons.append('surface_fit')
        if best['inlier_fraction']<cfg['min_inlier_fraction']: reasons.append('low_inlier_fraction')
        if best['key_points']<cfg['min_key_points'] or best['key_reverse_rms_m']>cfg['max_key_reverse_rms_m']:
            reasons.append('asymmetric_feature_not_resolved')
        if gap is not None and gap<cfg['ambiguity_score_gap_m']: reasons.append('pose_ambiguous')
    report=dict(valid=not reasons,reasons=reasons,frame='mid360s_lidar',model='known body/key/handle',
        control_authorized=False,grasp_ready=False,raw_points=len(points),roi_points=int(np.sum(roi)),
        voxel_points=len(p),remaining_points=len(remaining),removed_planes=planes,clusters=group_report,
        best=best,candidates=candidates[:8],elapsed_wall_s=time.perf_counter()-begin)
    report['placement_prior']=dict(up_lidar=up.tolist(),max_tilt_deg=cfg['upright_tilt_limit_deg'])
    return report,remaining,groups
