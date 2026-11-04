"""PNG annotations describe layout only; they never issue board commands."""
import json
def restored_layout(image):
 result=dict(source='opened_file',preview=False,metadata_trust='unverified_file_annotation')
 try:
  raw=image.info.get('RECTIFY_CAPTURE','')
  if not isinstance(raw,str) or len(raw)>1000000:return result
  document=json.loads(raw);capture=document.get('capture',{})
  if not isinstance(capture,dict):return result
  flags=capture.get('frame_geometry_flags')
  if type(flags) is int and flags>=0 and not flags&~0x3fe3:
   result.update(frame_geometry_flags=flags,display_layout='split' if flags&3==3 else 'full')
  elif capture.get('display_layout') in ('split','full'):result['display_layout']=capture['display_layout']
  if type(capture.get('frame_id')) is int:result['frame_id']=capture['frame_id']
 except (ValueError,TypeError,AttributeError):pass
 return result
