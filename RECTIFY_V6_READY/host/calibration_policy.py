"""Shared numeric acceptance policy, integrity binding and Brown-map certificate.

Engineering thresholds belong to this project, not to the contest rules.
The digest detects inconsistent edits; it is not a signature or a proof of accuracy.
"""
import hashlib,json,math

SCHEMA='RETURN_PC_1.brown.v2'
MODEL='brown_k1_k2_p1_p2'
MAX_RMS=1.0
MAX_HOLDOUT=1.5
MAX_PER_VIEW=2.0
MAX_FOCAL_RELATIVE_SIGMA=.10
MIN_VIEWS=12

def number(value,name):
    if isinstance(value,bool) or not isinstance(value,(int,float)):
        raise ValueError(name+'必须是有限数值')
    try:result=float(value)
    except (OverflowError,ValueError) as exc:raise ValueError(name+'数值范围无效') from exc
    if not math.isfinite(result):raise ValueError(name+'必须是有限数值')
    return result

def digest(profile):
    # Zoom is a runtime view choice. Camera geometry, calibration, quality and
    # provenance remain bound together even if a user changes runtime zoom.
    data={k:v for k,v in profile.items() if k not in ['integrity_sha256','zoom']}
    try:raw=json.dumps(data,ensure_ascii=False,sort_keys=True,separators=(',',':'),allow_nan=False).encode('utf-8')
    except (TypeError,ValueError) as exc:raise ValueError('标定内容包含不可编码或非有限字段') from exc
    return hashlib.sha256(raw).hexdigest()

def seal(profile):
    profile['integrity_sha256']=digest(profile)
    return profile

def quality_issues(q):
    """Evaluate measured values; the caller's accepted flag is not evidence."""
    if not isinstance(q,dict):return ['缺少质量报告']
    errors=[]
    try:
        rms=number(q['overall_reprojection_rms_pixels'],'整体RMS')
        limit=number(q['max_allowed_rms_pixels'],'整体RMS门槛')
        hold_limit=number(q['max_allowed_holdout_pixels'],'留出门槛')
        per_limit=number(q['max_allowed_per_view_pixels'],'单视图门槛')
        if not(0<limit<=MAX_RMS and 0<hold_limit<=MAX_HOLDOUT and 0<per_limit<=MAX_PER_VIEW):errors.append('报告门槛超出本工程策略')
        if rms<0 or rms>min(limit,MAX_RMS):errors.append('整体重投影RMS未达标')
        views=q['per_view'];held=q['held_out'];sigma=q['focal_relative_standard_deviation']
        if not isinstance(views,list) or len(views)<MIN_VIEWS:errors.append('有效视图数量不足')
        else:
            if len({v['image'] for v in views})!=len(views):errors.append('视图标识重复')
            hashes=[v['image_sha256'] for v in views]
            if any(not isinstance(h,str) or len(h)!=64 or any(c not in '0123456789abcdef' for c in h) for h in hashes) or len(set(hashes))!=len(hashes):errors.append('原图SHA缺失或内容重复')
            weight=0;sum_squared=0.
            for v in views:
                error=number(v['reprojection_rms_pixels'],'单视图RMS')
                n=v['corner_count']
                if type(n) is not int or not 12<=n<=1920*1080:raise ValueError('单视图角点数量无效')
                weight+=n;sum_squared+=n*error*error
                if error<0 or error>min(per_limit,MAX_PER_VIEW):errors.append('单视图重投影误差未达标')
            if abs(math.sqrt(sum_squared/weight)-rms)>1e-4:errors.append('整体RMS与逐视图加权RMS不一致')
        if q.get('heldout_validation')!='disjoint_view_and_pose_corners':errors.append('缺少独立角点留出验证')
        if not isinstance(held,list) or len(held)<2:errors.append('独立留出视图数量不足')
        else:
            view_names={v['image'] for v in views}
            if len({v['image'] for v in held})!=len(held) or any(v['image'] not in view_names for v in held):errors.append('留出视图标识不一致')
            for v in held:
                error=number(v['heldout_reprojection_rms_pixels'],'留出角点RMS')
                if error<0 or error>min(hold_limit,MAX_HOLDOUT):errors.append('留出角点误差未达标')
                if type(v['pose_fit_corner_count']) is not int or type(v['evaluation_corner_count']) is not int or min(v['pose_fit_corner_count'],v['evaluation_corner_count'])<6:errors.append('位姿／评估独立角点不足')
                elif v['pose_fit_corner_count']+v['evaluation_corner_count']!={p['image']:p['corner_count'] for p in views}.get(v['image']):errors.append('留出角点数量不一致')
        if not isinstance(sigma,list) or len(sigma)!=2:errors.append('焦距不确定度不完整')
        elif any(not 0<=number(v,'焦距相对不确定度')<=MAX_FOCAL_RELATIVE_SIGMA for v in sigma):errors.append('焦距不确定度未达标')
    except (KeyError,TypeError,ValueError) as exc:errors.append('质量报告字段缺失或无效：'+str(exc))
    return sorted(set(errors))

def _quad_min(a,b,c,maximum):
    values=[c,a*maximum*maximum+b*maximum+c]
    if a>0:
        critical=-b/(2*a)
        if 0<critical<maximum:values.append(a*critical*critical+b*critical+c)
    return min(values)

def mapping_certificate(words):
    """Conservative continuous-model non-folding bound over the entire image.

    The Brown normalized Jacobian is symmetric. Radial eigenvalues are A(r2)
    and A(r2)+2*r2*A'(r2). Bound tangential perturbation by its row-sum norm.
    A positive lower bound on the convex domain implies strict monotonicity.
    A failed certificate is not by itself proof of an actual fold.
    """
    cx=(words[2]>>2)/16384;cy=(words[3]>>2)/16384
    invx=words[8]/2**30;invy=words[9]/2**30
    # Include one Q14 LSB at the domain boundary. Zoom>=1 stays within it.
    x=(max(abs(cx),abs(1919-cx))+1/16384)*invx
    y=(max(abs(cy),abs(1079-cy))+1/16384)*invy
    r2=x*x+y*y;k1,k2,p1,p2=[v/2**24 for v in words[4:8]]
    radial=min(_quad_min(k2,k1,1.,r2),_quad_min(5*k2,3*k1,1.,r2))
    off=2*abs(p1)*x+2*abs(p2)*y
    tangent=max(2*abs(p1)*y+6*abs(p2)*x+off,6*abs(p1)*y+2*abs(p2)*x+off)
    lower=radial-tangent
    return dict(method='analytic_conservative_continuous_brown_jacobian',normalized_radius_squared_bound=r2,
                radial_eigenvalue_lower_bound=radial,tangential_norm_upper_bound=tangent,
                jacobian_eigenvalue_lower_bound=lower,minimum_required_margin=.001,certified_non_folding=lower>.001)
