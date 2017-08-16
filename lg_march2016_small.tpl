- meta:
    map: lg_march2016
    node: ChargingPoint1
    pointset: lg_march2016_small
    tag:
    - SafeNode
  node:
    edges:
    - action: undocking
      edge_id: ChargingPoint1_WayPoint13
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint13
      recovery_behaviours_config: ''
      top_vel: 0.55
    localise_by_topic: '{"topic": "/battery_state", "field": "charging", "val": true,
      "localise_anywhere": false}'
    map: lg_march2016
    name: ChargingPoint1
    pointset: lg_march2016_small
    pose:
      orientation:
        w: 0.698018193245
        x: -4.45035688301e-08
        y: -1.55396417956e-09
        z: 0.716080188751
      position:
        x: -3.30379867554
        y: 8.30790710449
        z: 0.0
    verts:
    - x: 0.667931556702
      y: 0.680954754353
    - x: -0.0750000029802
      y: 0.689999997616
    - x: -0.731799602509
      y: 0.698329746723
    - x: -0.864071846008
      y: -0.591036915779
    - x: -0.0750000029802
      y: -0.689999997616
    - x: 0.601235389709
      y: -0.733071565628
    xy_goal_tolerance: 0.0
    yaw_goal_tolerance: 0.0
- meta:
    map: lg_march2016
    node: Station
    pointset: lg_march2016_small
  node:
    edges:
    - action: move_base
      edge_id: Station_WayPoint1
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint1
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: Station_WayPoint4
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint4
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: Station_WayPoint17
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint17
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: Station_WayPoint10
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint10
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: Station_WayPoint5
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint5
      recovery_behaviours_config: ''
      top_vel: 0.55
    localise_by_topic: ''
    map: lg_march2016
    name: Station
    pointset: lg_march2016_small
    pose:
      orientation:
        w: 0.387936711311
        x: -1.6463889807e-08
        y: 3.29824189294e-07
        z: 0.921685993671
      position:
        x: 2.83139634132
        y: 4.92906808853
        z: 0.0
    verts:
    - x: 0.689999997616
      y: 0.287000000477
    - x: 0.287000000477
      y: 0.689999997616
    - x: -0.287000000477
      y: 0.689999997616
    - x: -0.689999997616
      y: 0.287000000477
    - x: -0.689999997616
      y: -0.287000000477
    - x: -0.287000000477
      y: -0.689999997616
    - x: 0.287000000477
      y: -0.689999997616
    - x: 0.689999997616
      y: -0.287000000477
    xy_goal_tolerance: 0.0
    yaw_goal_tolerance: 0.0
- meta:
    map: lg_march2016
    node: WayPoint1
    pointset: lg_march2016_small
  node:
    edges:
    - action: move_base
      edge_id: WayPoint1_Station
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: Station
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint1_WayPoint17
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint17
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: door_wait_and_pass
      edge_id: WayPoint1_WayPoint2
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint2
      recovery_behaviours_config: ''
      top_vel: 0.55
    localise_by_topic: ''
    map: lg_march2016
    name: WayPoint1
    pointset: lg_march2016_small
    pose:
      orientation:
        w: 0.737188696861
        x: -2.58611954074e-09
        y: 4.99746866112e-09
        z: -0.675686955452
      position:
        x: 1.39353239536
        y: 3.43237733841
        z: -6.93889390391e-18
    verts:
    - x: 0.689999997616
      y: 0.287000000477
    - x: 0.287000000477
      y: 0.689999997616
    - x: -0.287000000477
      y: 0.689999997616
    - x: -0.689999997616
      y: 0.287000000477
    - x: -0.689999997616
      y: -0.287000000477
    - x: -0.287000000477
      y: -0.689999997616
    - x: 0.287000000477
      y: -0.689999997616
    - x: 0.689999997616
      y: -0.287000000477
    xy_goal_tolerance: 0.3
    yaw_goal_tolerance: 0.1
- meta:
    map: lg_march2016
    node: WayPoint10
    pointset: lg_march2016_small
  node:
    edges:
    - action: move_base
      edge_id: WayPoint10_WayPoint4
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint4
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint10_Station
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: Station
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint10_WayPoint14
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint14
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint10_WayPoint5
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint5
      recovery_behaviours_config: ''
      top_vel: 0.55
    localise_by_topic: ''
    map: lg_march2016
    name: WayPoint10
    pointset: lg_march2016_small
    pose:
      orientation:
        w: 0.99718350172
        x: -3.08106140601e-09
        y: 1.17709797465e-09
        z: -0.0750008970499
      position:
        x: 4.50190353394
        y: 1.74884748459
        z: 0.0
    verts:
    - x: 0.689999997616
      y: 0.287000000477
    - x: 0.287000000477
      y: 0.689999997616
    - x: -0.287000000477
      y: 0.689999997616
    - x: -0.689999997616
      y: 0.287000000477
    - x: -0.689999997616
      y: -0.287000000477
    - x: -0.287000000477
      y: -0.689999997616
    - x: 0.287000000477
      y: -0.689999997616
    - x: 0.689999997616
      y: -0.287000000477
    xy_goal_tolerance: 0.3
    yaw_goal_tolerance: 0.1
- meta:
    map: lg_march2016
    node: WayPoint13
    pointset: lg_march2016_small
    tag: []
  node:
    edges:
    - action: docking
      edge_id: WayPoint13_ChargingPoint1
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: ChargingPoint1
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint13_WayPoint17
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint17
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint13_WayPoint19
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint19
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint13_WayPoint17_000
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint17
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint13_WayPoint18
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint18
      recovery_behaviours_config: ''
      top_vel: 0.55
    localise_by_topic: ''
    map: lg_march2016
    name: WayPoint13
    pointset: lg_march2016_small
    pose:
      orientation:
        w: 0.710120022297
        x: 8.30682189701e-09
        y: -2.03067305193e-08
        z: 0.704081237316
      position:
        x: -3.30279660225
        y: 7.07881450653
        z: 0.0
    verts:
    - x: 0.323300480843
      y: 0.383969783783
    - x: 0.287000000477
      y: 0.689999997616
    - x: -0.287000000477
      y: 0.689999997616
    - x: -0.689999997616
      y: 0.287000000477
    - x: -0.689999997616
      y: -0.287000000477
    - x: -0.287000000477
      y: -0.689999997616
    - x: 0.287000000477
      y: -0.689999997616
    - x: 0.319748044014
      y: -0.246846497059
    xy_goal_tolerance: 0.3
    yaw_goal_tolerance: 0.1
- meta:
    map: lg_march2016
    node: WayPoint14
    pointset: lg_march2016_small
  node:
    edges:
    - action: move_base
      edge_id: WayPoint14_WayPoint10
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint10
      recovery_behaviours_config: ''
      top_vel: 0.55
    localise_by_topic: ''
    map: lg_march2016
    name: WayPoint14
    pointset: lg_march2016_small
    pose:
      orientation:
        w: 0.69693505764
        x: -4.0081085e-09
        y: 3.7042394e-09
        z: -0.717134296894
      position:
        x: 4.37592697144
        y: -1.75717246532
        z: 0.0
    verts:
    - x: 0.689999997616
      y: 0.287000000477
    - x: 0.287000000477
      y: 0.689999997616
    - x: -0.287000000477
      y: 0.689999997616
    - x: -0.689999997616
      y: 0.287000000477
    - x: -0.689999997616
      y: -0.287000000477
    - x: -0.287000000477
      y: -0.689999997616
    - x: 0.287000000477
      y: -0.689999997616
    - x: 0.630312204361
      y: -0.350662589073
    xy_goal_tolerance: 0.3
    yaw_goal_tolerance: 0.1
- meta:
    map: lg_march2016
    node: WayPoint17
    pointset: lg_march2016_small
    tag:
    - Exploration
  node:
    edges:
    - action: move_base
      edge_id: WayPoint17_WayPoint1
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint1
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint17_WayPoint4
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint4
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint17_Station
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: Station
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint17_WayPoint18
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint18
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint17_WayPoint13
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint13
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint17_WayPoint5
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint5
      recovery_behaviours_config: ''
      top_vel: 0.55
    localise_by_topic: ''
    map: lg_march2016
    name: WayPoint17
    pointset: lg_march2016_small
    pose:
      orientation:
        w: 0.699821949005
        x: 3.14223802533e-09
        y: -2.18831330834e-09
        z: -0.714317381382
      position:
        x: -0.51407623291
        y: 6.10357570648
        z: -6.93889390391e-18
    verts:
    - x: 0.689999997616
      y: 0.287000000477
    - x: 0.287000000477
      y: 0.689999997616
    - x: -0.287000000477
      y: 0.689999997616
    - x: -0.689999997616
      y: 0.287000000477
    - x: -0.689999997616
      y: -0.287000000477
    - x: -0.287000000477
      y: -0.689999997616
    - x: 0.287000000477
      y: -0.689999997616
    - x: 0.689999997616
      y: -0.287000000477
    xy_goal_tolerance: 0.3
    yaw_goal_tolerance: 0.1
- meta:
    map: lg_march2016
    node: WayPoint18
    pointset: lg_march2016_small
    tag:
    - Exploration
  node:
    edges:
    - action: move_base
      edge_id: WayPoint18_WayPoint17
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint17
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint18_WayPoint13
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint13
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint18_WayPoint19
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint19
      recovery_behaviours_config: ''
      top_vel: 0.55
    localise_by_topic: ''
    map: lg_march2016
    name: WayPoint18
    pointset: lg_march2016_small
    pose:
      orientation:
        w: 0.727384209633
        x: 6.92142676506e-10
        y: -2.96611046835e-09
        z: -0.68623059988
      position:
        x: -3.32575440407
        y: 6.28892326355
        z: 6.93889390391e-18
    verts:
    - x: 0.689999997616
      y: 0.287000000477
    - x: 0.287000000477
      y: 0.689999997616
    - x: -0.287000000477
      y: 0.689999997616
    - x: -0.689999997616
      y: 0.287000000477
    - x: -0.689999997616
      y: -0.287000000477
    - x: -0.287000000477
      y: -0.689999997616
    - x: 0.287000000477
      y: -0.689999997616
    - x: 0.689999997616
      y: -0.287000000477
    xy_goal_tolerance: 0.3
    yaw_goal_tolerance: 0.1
- meta:
    map: lg_march2016
    node: WayPoint19
    pointset: lg_march2016_small
    tag:
    - Exploration
  node:
    edges:
    - action: move_base
      edge_id: WayPoint19_WayPoint20
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint20
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint19_WayPoint13
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint13
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint19_WayPoint18
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint18
      recovery_behaviours_config: ''
      top_vel: 0.55
    localise_by_topic: ''
    map: lg_march2016
    name: WayPoint19
    pointset: lg_march2016_small
    pose:
      orientation:
        w: 0.718959450722
        x: -4.82604045615e-09
        y: 3.43834183347e-10
        z: -0.69505238533
      position:
        x: -6.24211883545
        y: 6.26723670959
        z: 0.0
    verts:
    - x: 0.689999997616
      y: 0.287000000477
    - x: 0.287000000477
      y: 0.689999997616
    - x: -0.287000000477
      y: 0.689999997616
    - x: -0.689999997616
      y: 0.287000000477
    - x: -0.689999997616
      y: -0.287000000477
    - x: -0.287000000477
      y: -0.689999997616
    - x: 0.287000000477
      y: -0.689999997616
    - x: 0.689999997616
      y: -0.287000000477
    xy_goal_tolerance: 0.3
    yaw_goal_tolerance: 0.1
- meta:
    map: lg_march2016
    node: WayPoint2
    pointset: lg_march2016_small
  node:
    edges:
    - action: move_base
      edge_id: WayPoint2_WayPoint3
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint3
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: door_wait_and_pass
      edge_id: WayPoint2_WayPoint1
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint1
      recovery_behaviours_config: ''
      top_vel: 0.55
    localise_by_topic: ''
    map: lg_march2016
    name: WayPoint2
    pointset: lg_march2016_small
    pose:
      orientation:
        w: 0.662158846855
        x: -6.40184083522e-09
        y: -6.64776766968e-09
        z: 0.749363541603
      position:
        x: 1.50039124489
        y: 1.22151339054
        z: 0.0
    verts:
    - x: 0.689999997616
      y: 0.287000000477
    - x: 0.287000000477
      y: 0.689999997616
    - x: -0.287000000477
      y: 0.689999997616
    - x: -0.689999997616
      y: 0.287000000477
    - x: -0.689999997616
      y: -0.287000000477
    - x: -0.287000000477
      y: -0.689999997616
    - x: 0.287000000477
      y: -0.689999997616
    - x: 0.689999997616
      y: -0.287000000477
    xy_goal_tolerance: 0.3
    yaw_goal_tolerance: 0.1
- meta:
    map: lg_march2016
    node: WayPoint20
    pointset: lg_march2016_small
    tag:
    - Exploration
  node:
    edges:
    - action: move_base
      edge_id: WayPoint20_WayPoint19
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint19
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint20_WayPoint21
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint21
      recovery_behaviours_config: ''
      top_vel: 0.55
    localise_by_topic: ''
    map: lg_march2016
    name: WayPoint20
    pointset: lg_march2016_small
    pose:
      orientation:
        w: 0.63615322113
        x: 3.02332692215e-09
        y: -1.44375036371e-08
        z: -0.771562993526
      position:
        x: -8.8519153595
        y: 6.39514636993
        z: -3.46944695195e-18
    verts:
    - x: 0.689999997616
      y: 0.287000000477
    - x: 0.287000000477
      y: 0.689999997616
    - x: -0.287000000477
      y: 0.689999997616
    - x: -0.689999997616
      y: 0.287000000477
    - x: -0.689999997616
      y: -0.287000000477
    - x: -0.287000000477
      y: -0.689999997616
    - x: 0.287000000477
      y: -0.689999997616
    - x: 0.689999997616
      y: -0.287000000477
    xy_goal_tolerance: 0.3
    yaw_goal_tolerance: 0.1
- meta:
    map: lg_march2016
    node: WayPoint21
    pointset: lg_march2016_small
    tag:
    - ExplorationGround
  node:
    edges:
    - action: move_base
      edge_id: WayPoint21_WayPoint20
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint20
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint21_WayPoint22
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint22
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint21_WayPoint25
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint25
      recovery_behaviours_config: ''
      top_vel: 0.55
    localise_by_topic: ''
    map: lg_march2016
    name: WayPoint21
    pointset: lg_march2016_small
    pose:
      orientation:
        w: 0.720213890076
        x: -1.92053017933e-09
        y: -7.03850266959e-09
        z: -0.693752229214
      position:
        x: -11.9754524231
        y: 6.64629554749
        z: -3.46944695195e-18
    verts:
    - x: 0.689999997616
      y: 0.287000000477
    - x: 0.287000000477
      y: 0.689999997616
    - x: -0.287000000477
      y: 0.689999997616
    - x: -0.689999997616
      y: 0.287000000477
    - x: -0.689999997616
      y: -0.287000000477
    - x: -0.287000000477
      y: -0.689999997616
    - x: 0.287000000477
      y: -0.689999997616
    - x: 0.689999997616
      y: -0.287000000477
    xy_goal_tolerance: 0.3
    yaw_goal_tolerance: 0.1
- meta:
    map: lg_march2016
    node: WayPoint22
    pointset: lg_march2016_small
  node:
    edges:
    - action: move_base
      edge_id: WayPoint22_WayPoint21
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint21
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint22_WayPoint23
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint23
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint22_WayPoint24
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint24
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint22_WayPoint25
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint25
      recovery_behaviours_config: ''
      top_vel: 0.55
    localise_by_topic: ''
    map: lg_march2016
    name: WayPoint22
    pointset: lg_march2016_small
    pose:
      orientation:
        w: 0.999753475189
        x: 0.0
        y: 0.0
        z: -0.0222043767571
      position:
        x: -15.327914238
        y: 7.23445987701
        z: 3.46944695195e-18
    verts:
    - x: 0.689999997616
      y: 0.287000000477
    - x: 0.287000000477
      y: 0.689999997616
    - x: -0.287000000477
      y: 0.689999997616
    - x: -0.689999997616
      y: 0.287000000477
    - x: -0.689999997616
      y: -0.287000000477
    - x: -0.287000000477
      y: -0.689999997616
    - x: 0.287000000477
      y: -0.689999997616
    - x: 0.689999997616
      y: -0.287000000477
    xy_goal_tolerance: 0.3
    yaw_goal_tolerance: 0.1
- meta:
    map: lg_march2016
    node: WayPoint23
    pointset: lg_march2016_small
  node:
    edges:
    - action: move_base
      edge_id: WayPoint23_WayPoint22
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint22
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint23_WayPoint24
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint24
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint23_WayPoint25
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint25
      recovery_behaviours_config: ''
      top_vel: 0.55
    localise_by_topic: ''
    map: lg_march2016
    name: WayPoint23
    pointset: lg_march2016_small
    pose:
      orientation:
        w: 0.705667513383
        x: 0.0
        y: 0.0
        z: -0.708543125403
      position:
        x: -17.324968177
        y: 4.53167754063
        z: 3.46944695195e-18
    verts:
    - x: 0.689999997616
      y: 0.287000000477
    - x: 0.287000000477
      y: 0.689999997616
    - x: -0.287000000477
      y: 0.689999997616
    - x: -0.689999997616
      y: 0.287000000477
    - x: -0.689999997616
      y: -0.287000000477
    - x: -0.287000000477
      y: -0.689999997616
    - x: 0.287000000477
      y: -0.689999997616
    - x: 0.689999997616
      y: -0.287000000477
    xy_goal_tolerance: 0.3
    yaw_goal_tolerance: 0.1
- meta:
    map: lg_march2016
    node: WayPoint24
    pointset: lg_march2016_small
  node:
    edges:
    - action: move_base
      edge_id: WayPoint24_WayPoint22
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint22
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint24_WayPoint23
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint23
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint24_WayPoint26
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint26
      recovery_behaviours_config: ''
      top_vel: 0.55
    localise_by_topic: ''
    map: lg_march2016
    name: WayPoint24
    pointset: lg_march2016_small
    pose:
      orientation:
        w: 0.715275168419
        x: -2.6008548093e-10
        y: 7.40224270768e-10
        z: 0.698843181133
      position:
        x: -16.9890880585
        y: 8.61416816711
        z: -3.46944695195e-18
    verts:
    - x: 0.689999997616
      y: 0.287000000477
    - x: 0.287000000477
      y: 0.689999997616
    - x: -0.287000000477
      y: 0.689999997616
    - x: -0.689999997616
      y: 0.287000000477
    - x: -0.689999997616
      y: -0.287000000477
    - x: -0.287000000477
      y: -0.689999997616
    - x: 0.287000000477
      y: -0.689999997616
    - x: 0.689999997616
      y: -0.287000000477
    xy_goal_tolerance: 0.3
    yaw_goal_tolerance: 0.1
- meta:
    map: lg_march2016
    node: WayPoint25
    pointset: lg_march2016_small
  node:
    edges:
    - action: move_base
      edge_id: WayPoint25_WayPoint21
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint21
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint25_WayPoint22
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint22
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint25_WayPoint23
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint23
      recovery_behaviours_config: ''
      top_vel: 0.55
    localise_by_topic: ''
    map: lg_march2016
    name: WayPoint25
    pointset: lg_march2016_small
    pose:
      orientation:
        w: 0.697034925531
        x: 0.0
        y: 0.0
        z: -0.717037176574
      position:
        x: -14.8068789893
        y: 5.11016296041
        z: 6.93889390391e-18
    verts:
    - x: 0.689999997616
      y: 0.287000000477
    - x: 0.287000000477
      y: 0.689999997616
    - x: -0.287000000477
      y: 0.689999997616
    - x: -0.689999997616
      y: 0.287000000477
    - x: -0.689999997616
      y: -0.287000000477
    - x: -0.287000000477
      y: -0.689999997616
    - x: 0.287000000477
      y: -0.689999997616
    - x: 0.689999997616
      y: -0.287000000477
    xy_goal_tolerance: 0.3
    yaw_goal_tolerance: 0.1
- meta:
    map: lg_march2016
    node: WayPoint26
    pointset: lg_march2016_small
  node:
    edges:
    - action: move_base
      edge_id: WayPoint26_WayPoint24
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint24
      recovery_behaviours_config: ''
      top_vel: 0.55
    localise_by_topic: ''
    map: lg_march2016
    name: WayPoint26
    pointset: lg_march2016_small
    pose:
      orientation:
        w: 0.99988424778
        x: 4.1138226e-09
        y: 2.802784e-09
        z: -0.0152153624222
      position:
        x: -16.8391513824
        y: 12.7726173401
        z: -0.0
    verts:
    - x: 0.266834259033
      y: 0.313598632812
    - x: 0.287000000477
      y: 0.689999997616
    - x: -0.287000000477
      y: 0.689999997616
    - x: -0.689999997616
      y: 0.287000000477
    - x: -0.689999997616
      y: -0.287000000477
    - x: -0.287000000477
      y: -0.689999997616
    - x: 0.287000656128
      y: -0.689999580383
    - x: 0.254169464111
      y: -0.217167854309
    xy_goal_tolerance: 0.3
    yaw_goal_tolerance: 0.1
- meta:
    map: lg_march2016
    node: WayPoint3
    pointset: lg_march2016_small
    tag:
    - ExplorationGround
    - SafeNode
  node:
    edges:
    - action: move_base
      edge_id: WayPoint3_WayPoint2
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint2
      recovery_behaviours_config: ''
      top_vel: 0.55
    localise_by_topic: ''
    map: lg_march2016
    name: WayPoint3
    pointset: lg_march2016_small
    pose:
      orientation:
        w: 0.704930067062
        x: 4.22868993155e-08
        y: 1.31020883032e-08
        z: -0.709276854992
      position:
        x: 0.0611273497343
        y: -0.791955590248
        z: 6.93889390391e-18
    verts:
    - x: 0.689999997616
      y: 0.287000000477
    - x: 0.287000000477
      y: 0.689999997616
    - x: -0.287000000477
      y: 0.689999997616
    - x: -0.689999997616
      y: 0.287000000477
    - x: -0.689999997616
      y: -0.287000000477
    - x: -0.287000000477
      y: -0.689999997616
    - x: 0.287000000477
      y: -0.689999997616
    - x: 0.689999997616
      y: -0.287000000477
    xy_goal_tolerance: 0.3
    yaw_goal_tolerance: 0.1
- meta:
    map: lg_march2016
    node: WayPoint4
    pointset: lg_march2016_small
    tag:
    - Exploration
  node:
    edges:
    - action: move_base
      edge_id: WayPoint4_Station
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: Station
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint4_WayPoint5
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint5
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint4_WayPoint10
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint10
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint4_WayPoint17
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint17
      recovery_behaviours_config: ''
      top_vel: 0.55
    localise_by_topic: ''
    map: lg_march2016
    name: WayPoint4
    pointset: lg_march2016_small
    pose:
      orientation:
        w: 0.0862110704184
        x: -8.58735660358e-08
        y: 4.25640358515e-08
        z: -0.996277093887
      position:
        x: 87.6560440063
        y: -2552.65600586
        z: 0.0
    verts:
    - x: 0.689999997616
      y: 0.287000000477
    - x: 0.287000000477
      y: 0.689999997616
    - x: -0.287000000477
      y: 0.689999997616
    - x: -0.689999997616
      y: 0.287000000477
    - x: -0.689999997616
      y: -0.287000000477
    - x: -0.287000000477
      y: -0.689999997616
    - x: 0.287000000477
      y: -0.689999997616
    - x: 0.689999997616
      y: -0.287000000477
    xy_goal_tolerance: 0.3
    yaw_goal_tolerance: 0.1
- meta:
    map: lg_march2016
    node: WayPoint5
    pointset: lg_march2016_small
  node:
    edges:
    - action: move_base
      edge_id: WayPoint5_WayPoint4
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint4
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint5_WayPoint6
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint6
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint5_WayPoint10
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint10
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint5_Station
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: Station
      recovery_behaviours_config: ''
      top_vel: 0.55
    - action: move_base
      edge_id: WayPoint5_WayPoint17
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint17
      recovery_behaviours_config: ''
      top_vel: 0.55
    localise_by_topic: ''
    map: lg_march2016
    name: WayPoint5
    pointset: lg_march2016_small
    pose:
      orientation:
        w: 0.368227690458
        x: 1.87593829359e-09
        y: -1.07243653957e-08
        z: 0.929736077785
      position:
        x: 4.52137327194
        y: 6.66267251968
        z: 3.46944695195e-18
    verts:
    - x: 0.689999997616
      y: 0.287000000477
    - x: 0.287000000477
      y: 0.689999997616
    - x: -0.287000000477
      y: 0.689999997616
    - x: -0.690135478973
      y: 0.283159255981
    - x: -0.689999997616
      y: -0.287000000477
    - x: -0.287000000477
      y: -0.689999997616
    - x: 0.287000000477
      y: -0.689999997616
    - x: 0.689999997616
      y: -0.287000000477
    xy_goal_tolerance: 0.3
    yaw_goal_tolerance: 0.1
- meta:
    map: lg_march2016
    node: WayPoint6
    pointset: lg_march2016_small
  node:
    edges:
    - action: move_base
      edge_id: WayPoint6_WayPoint5
      inflation_radius: 0.0
      map_2d: lg_march2016
      node: WayPoint5
      recovery_behaviours_config: ''
      top_vel: 0.55
    localise_by_topic: ''
    map: lg_march2016
    name: WayPoint6
    pointset: lg_march2016_small
    pose:
      orientation:
        w: 0.72290045023
        x: 2.22864449118e-10
        y: -3.0613065416e-09
        z: 0.690952599049
      position:
        x: 4.95487880707
        y: 12.3036756516
        z: -3.46944695195e-18
    verts:
    - x: 0.689999997616
      y: 0.287000000477
    - x: 0.287000000477
      y: 0.689999997616
    - x: -0.287000000477
      y: 0.689999997616
    - x: -0.689999997616
      y: 0.287000000477
    - x: -0.689999997616
      y: -0.287000000477
    - x: -0.287000000477
      y: -0.689999997616
    - x: 0.287000000477
      y: -0.689999997616
    - x: 0.689999997616
      y: -0.287000000477
    xy_goal_tolerance: 0.3
    yaw_goal_tolerance: 0.1
