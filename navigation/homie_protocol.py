"""Identical body-yaw velocity commands for Isaac Sim and MuJoCo comparisons."""
PROTOCOL=[('settle',5.,(0.,0.,0.)),('stand',10.,(0.,0.,0.))]
for name,seconds,command in (
    ('straight',12.,(.3,0.,0.)),('slow',10.,(.1,0.,0.)),('crawl',10.,(.05,0.,0.)),
    ('turn_positive',10.,(0.,0.,.4)),('turn_negative',10.,(0.,0.,-.4)),
    ('small_turn_positive',10.,(0.,0.,.15)),('small_turn_negative',10.,(0.,0.,-.15))):
    PROTOCOL.extend([(name,seconds,command),('stop_'+name,5.,(0.,0.,0.))])

# Real-navigation command envelope: vx <= 0.15 m/s, |wz| <= 0.25 rad/s.
NAV2_PROTOCOL=[('settle',5.,(0.,0.,0.)),('stand',10.,(0.,0.,0.))]
for name,command in (
    ('forward_005',(.05,0.,0.)),('forward_010',(.1,0.,0.)),('forward_015',(.15,0.,0.)),
    ('yaw_p005',(0.,0.,.05)),('yaw_n005',(0.,0.,-.05)),
    ('yaw_p015',(0.,0.,.15)),('yaw_n015',(0.,0.,-.15)),
    ('yaw_p025',(0.,0.,.25)),('yaw_n025',(0.,0.,-.25))):
    NAV2_PROTOCOL.extend([(name,12.,command),('stop_'+name,5.,(0.,0.,0.))])
