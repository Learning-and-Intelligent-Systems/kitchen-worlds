(define (stream pr2-tamp)
  (:stream sample-pose
    :inputs (?o ?r)
    :domain (Stackable ?o ?r)
    :outputs (?p)
    :certified (and (Pose ?o ?p) (Supported ?o ?p ?r))
  )
  (:stream sample-grasp
    :inputs (?o)
    :domain (Graspable ?o)
    :outputs (?g)
    :certified (Grasp ?o ?g)
  )

  ;(:stream inverse-kinematics
  ;  :inputs (?a ?o ?p ?g)
  ;  :domain (and (Controllable ?a) (Pose ?o ?p) (Grasp ?o ?g))
  ;  :outputs (?q ?t)
  ;  :certified (and (BConf ?q) (ATraj ?t) (Kin ?a ?o ?p ?g ?q ?t))
  ;)
  ;(:stream inverse-kinematics-wconf
  ;  :inputs (?a ?o ?p ?g ?w)
  ;  :domain (and (Controllable ?a) (Pose ?o ?p) (Grasp ?o ?g) (WConf ?w))
  ;  :outputs (?q ?t)
  ;  :certified (and (BConf ?q) (ATraj ?t) (KinWConf ?a ?o ?p ?g ?q ?t ?w))
  ;)
  (:stream inverse-reachability-wconf
    :inputs (?a ?o ?p ?g ?w)
    :domain (and (Controllable ?a) (Pose ?o ?p) (Grasp ?o ?g) (WConf ?w))
    :outputs (?q)
    :certified (and (BConf ?q) (ReachWConf ?a ?o ?p ?g ?q ?w))
  )
  (:stream inverse-kinematics-wconf
    :inputs (?a ?o ?p ?g ?q ?w)
    :domain (ReachWConf ?a ?o ?p ?g ?q ?w)
    :fluents (AtPose AtPosition) ; AtGrasp AtBConf AtAConf
    :outputs (?t)
    :certified (and (ATraj ?t) (KinWConf ?a ?o ?p ?g ?q ?t ?w))
  )

  ;(:stream plan-base-motion
  ;  :inputs (?q1 ?q2)
  ;  :domain (and (BConf ?q1) (BConf ?q2))
  ;  ;:fluents (AtPose AtGrasp) ; AtBConf
  ;  :outputs (?t)
  ;  :certified (and (BTraj ?t) (BaseMotion ?q1 ?t ?q2))
  ;)
  ;; only when using wconf
  (:stream plan-base-motion-wconf
    :inputs (?q1 ?q2 ?w)
    :domain (and (BConf ?q1) (BConf ?q2) (WConf ?w))
    :fluents (AtPose AtGrasp AtPosition AtAConf) ; AtBConf AtAConf
    :outputs (?t)
    :certified (and (BTraj ?t) (BaseMotionWConf ?q1 ?t ?q2 ?w))
  )

  (:stream test-cfree-pose-pose
    :inputs (?o1 ?p1 ?o2 ?p2)
    :domain (and (Pose ?o1 ?p1) (Pose ?o2 ?p2))
    :certified (CFreePosePose ?o1 ?p1 ?o2 ?p2)
  )
  (:stream test-cfree-approach-pose
    :inputs (?o1 ?p1 ?g1 ?o2 ?p2)
    :domain (and (Pose ?o1 ?p1) (Grasp ?o1 ?g1) (Pose ?o2 ?p2))
    :certified (CFreeApproachPose ?o1 ?p1 ?g1 ?o2 ?p2)
  )
  ; TODO(caelan): conf collision streams
  ;(:stream test-cfree-traj-pose
  ;  :inputs (?t ?o2 ?p2)
  ;  :domain (and (ATraj ?t) (Pose ?o2 ?p2))
  ;  :certified (CFreeTrajPose ?t ?o2 ?p2)
  ;)
  ;(:stream test-cfree-traj-position
  ;  :inputs (?t ?o2 ?p2)
  ;  :domain (and (ATraj ?t) (Position ?o2 ?p2))
  ;  :certified (CFreeTrajPosition ?t ?o2 ?p2)
  ;)
  ;(:stream test-cfree-btraj-pose
  ;  :inputs (?t ?o2 ?p2)
  ;  :domain (and (BTraj ?t) (Pose ?o2 ?p2))
  ;  :certified (CFreeBTrajPose ?t ?o2 ?p2)
  ;)

  ;(:stream test-pose-in-space
  ;  :inputs (?o ?p ?r)
  ;  :domain (and (Containable ?o ?r) (Pose ?o ?p))
  ;  :certified (and (Contained ?o ?p ?r))
  ;)
  (:stream sample-pose-inside
    :inputs (?o ?r)
    :domain (Containable ?o ?r)
    :outputs (?p)
    :certified (and (Pose ?o ?p) (Contained ?o ?p ?r))
  )
  (:stream get-joint-position-open
    :inputs (?o ?p1)
    :domain (and (Joint ?o) (Position ?o ?p1) (IsClosedPosition ?o ?p1))
    :outputs (?p2)
    :certified (and (Position ?o ?p2) (IsOpenedPosition ?o ?p2))
  )

  ;(:stream sample-joint-position-open
  ;  :inputs (?o)
  ;  :domain (and (Joint ?o))
  ;  :outputs (?p2)
  ;  :certified (and (Position ?o ?p2) (IsOpenedPosition ?o ?p2))
  ;)
  ;(:stream sample-joint-position-closed
  ;  :inputs (?o)
  ;  :domain (and (Joint ?o))
  ;  :outputs (?p2)
  ;  :certified (and (Position ?o ?p2) (IsClosedPosition ?o ?p2))
  ;)
  ;(:stream test-joint-position-open
  ;  :inputs (?o ?p)
  ;  :domain (and (Joint ?o) (Position ?o ?p))
  ;  :certified (IsOpenedPosition ?o ?p)
  ;)
  ;(:stream test-joint-position-closed
  ;  :inputs (?o ?p)
  ;  :domain (and (Joint ?o) (Position ?o ?p))
  ;  :certified (IsClosedPosition ?o ?p)
  ;)

    (:stream sample-handle-grasp
      :inputs (?o)
      :domain (Joint ?o)
      :outputs (?g)
      :certified (HandleGrasp ?o ?g)
    )
    (:stream inverse-kinematics-grasp-handle
      :inputs (?a ?o ?p ?g)
      :domain (and (Controllable ?a) (Position ?o ?p) (HandleGrasp ?o ?g) (IsClosedPosition ?o ?p))
      :outputs (?q ?aq ?t)
      :certified (and (BConf ?q) (AConf ?a ?aq) (ATraj ?t)
                      (GraspHandle ?a ?o ?p ?g ?q ?aq)
                      (KinGraspHandle ?a ?o ?p ?g ?q ?aq ?t))
    )
    (:stream inverse-kinematics-ungrasp-handle
      :inputs (?a ?o ?p ?g ?q ?aq1)
      ;:domain (and (Controllable ?a) (Position ?o ?p) (HandleGrasp ?o ?g) (BConf ?q) (AConf ?a ?aq1) (IsOpenedPosition ?o ?p))
      :domain (and (UngraspHandle ?a ?o ?p ?g ?q ?aq1) (IsOpenedPosition ?o ?p))
      :outputs (?aq2 ?t)
      :certified (and (AConf ?a ?aq2) (ATraj ?t)
                      (KinUngraspHandle ?a ?o ?p ?g ?q ?aq1 ?aq2 ?t))
    )

    (:stream plan-base-pull-drawer-handle
      :inputs (?a ?o ?p1 ?p2 ?g ?q1)
      :domain (and (Controllable ?a) (Drawer ?o) (Position ?o ?p1) (Position ?o ?p2) (HandleGrasp ?o ?g) (BConf ?q1))
      :outputs (?q2 ?t)
      :certified (and (BConf ?q2) (BTraj ?t) (KinPullDrawerHandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?t))
    )
    (:stream plan-base-pull-door-handle
      ;:inputs (?a ?o ?p1 ?p2 ?g ?q1 ?aq1)
      ;:domain (and (Controllable ?a) (Door ?o) (Position ?o ?p1) (Position ?o ?p2) (HandleGrasp ?o ?g)
      ;             (BConf ?q1) (AConf ?a ?aq1) (IsClosedPosition ?o ?p1) (IsOpenedPosition ?o ?p2))
      ;:inputs (?a ?o ?p1 ?p2 ?g ?q1 ?aq1 ?t1)
      ;:domain (and (KinGraspHandle ?a ?o ?p1 ?g ?q1 ?aq1 ?t1) (Position ?o ?p2) (IsOpenedPosition ?o ?p2))
      :inputs (?a ?o ?p1 ?p2 ?g ?q1 ?aq1)
      :domain (and (GraspHandle ?a ?o ?p1 ?g ?q1 ?aq1) (Position ?o ?p2) (IsOpenedPosition ?o ?p2))
      :outputs (?q2 ?bt ?aq2 ?at)
      :certified (and (BConf ?q2) (UngraspBConf ?q2) (BTraj ?bt) (AConf ?a ?aq2) (ATraj ?at)
                      (UngraspHandle ?a ?o ?p2 ?g ?q2 ?aq2)
                      (KinPullDoorHandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?bt ?aq1 ?aq2 ?at))
    )
    (:stream plan-arm-turn-knob-handle
      :inputs (?a ?o ?p1 ?p2 ?g ?q ?aq1)
      :domain (and (Controllable ?a) (Knob ?o) (Position ?o ?p1) (Position ?o ?p2) (HandleGrasp ?o ?g)
                   (BConf ?q) (AConf ?a ?aq1) (IsClosedPosition ?o ?p1) (IsOpenedPosition ?o ?p2))
      :outputs (?aq2 ?at)
      :certified (and (AConf ?a ?aq2) (ATraj ?at) (KinTurnKnob ?a ?o ?p1 ?p2 ?g ?q ?aq1 ?aq2 ?at))
    )

    (:stream sample-marker-grasp
      :inputs (?o)
      :domain (and (Marker ?o)) ;; (Graspable ?o)
      :outputs (?g)
      :certified (MarkerGrasp ?o ?g)
    )
    (:stream sample-marker-pose
      :inputs (?o ?p1)
      :domain (and (Marker ?o) (Pose ?o ?p1))
      :outputs (?p2)
      :certified (Pose ?o ?p2)
    )
    (:stream inverse-kinematics-grasp-marker
      :inputs (?a ?o ?p ?g)
      :domain (and (Controllable ?a) (Pose ?o ?p) (MarkerGrasp ?o ?g))
      :outputs (?q ?t)
      :certified (and (BConf ?q) (ATraj ?t) (KinGraspMarker ?a ?o ?p ?g ?q ?t))
    )
    (:stream inverse-kinematics-ungrasp-marker
      :inputs (?a ?o ?p ?g ?q)
      :domain (and (Controllable ?a) (Pose ?o ?p) (MarkerGrasp ?o ?g) (BConf ?q))
      :outputs (?t)
      :certified (and (ATraj ?t) (KinUngraspMarker ?a ?o ?p ?g ?q ?t))
    )
    (:stream plan-base-pull-marker-random
      :inputs (?a ?o ?p1 ?g ?q1 ?o2 ?p3)
      :domain (and (Controllable ?a) (Marker ?o) (Pose ?o ?p1) (MarkerGrasp ?o ?g) (BConf ?q1) (Cart ?o2) (Pose ?o2 ?p3))
      :outputs (?p2 ?q2 ?p4 ?t)
      :certified (and (Pose ?o ?p2) (BConf ?q2) (Pose ?o2 ?p4) (BTraj ?t)
                      (KinPullMarkerRandom ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?o2 ?p3 ?p4 ?t))
    )
    ;(:stream plan-base-pull-marker-to-pose
    ;  :inputs (?a ?o ?p1 ?p2 ?g ?q1 ?o2 ?p3)
    ;  :domain (and (Controllable ?a) (Marker ?o) (Pose ?o ?p1) (Pose ?o ?p2) (MarkerGrasp ?o ?g) (BConf ?q1) (Cart ?o2) (Pose ?o2 ?p3))
    ;  :outputs (?q2 ?p4 ?t)
    ;  :certified (and (BConf ?q2) (Pose ?o2 ?p4) (BTraj ?t) (KinPullMarkerToPose ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?o2 ?p3 ?p4 ?t))
    ;)
    ;(:stream plan-base-pull-marker-to-bconf
    ;  :inputs (?a ?o ?p1 ?g ?q1 ?q2 ?o2 ?p3)
    ;  :domain (and (Controllable ?a) (Marker ?o) (Pose ?o ?p1) (MarkerGrasp ?o ?g) (BConf ?q1) (BConf ?q2) (Cart ?o2) (Pose ?o2 ?p3))
    ;  :outputs (?p2 ?p4 ?t)
    ;  :certified (and (Pose ?o ?p2) (Pose ?o2 ?p4) (BTraj ?t) (KinPullMarkerToBConf ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?o2 ?p3 ?p4 ?t))
    ;)

  (:stream sample-bconf-in-region
    :inputs (?r)
    :domain (and (Environment ?r))
    :outputs (?q)
    :certified (and (BConf ?q) (BConfInRegion ?q ?r))
  )
  (:stream sample-pose-in-region
    :inputs (?o ?r)
    :domain (and (Moveable ?o) (Environment ?r))
    :outputs (?p)
    :certified (and (Pose ?o ?p) (PoseInRegion ?o ?p ?r))
  )

  (:stream test-bconf-in-region
    :inputs (?q ?r)
    :domain (and (BConf ?q) (Environment ?r))
    :certified (BConfInRegion ?q ?r)
  )
  (:stream test-pose-in-region
    :inputs (?o ?p ?r)
    :domain (and (Pose ?o ?p) (Environment ?r))
    :certified (PoseInRegion ?o ?p ?r)
  )

    ;(:stream update-wconf-p
    ;  :inputs (?w1 ?o ?p)
    ;  :domain (and (WConf ?w1) (Pose ?o ?p))
    ;  :outputs (?w2)
    ;  :certified (and (WConf ?w2) (NewWConfP ?w1 ?o ?p ?w2))
    ;)
    ;(:stream update-wconf-p-two
    ;  :inputs (?w1 ?o ?p ?o2 ?p2)
    ;  :domain (and (WConf ?w1) (Pose ?o ?p) (Pose ?o2 ?p2))
    ;  :outputs (?w2)
    ;  :certified (and (WConf ?w2) (NewWConfPTwo ?w1 ?o ?p ?o2 ?p2 ?w2))
    ;)
    ;(:stream update-wconf-pst
    ;  :inputs (?w1 ?o ?pst)
    ;  :domain (and (WConf ?w1) (Position ?o ?pst))
    ;  :outputs (?w2)
    ;  :certified (and (WConf ?w2) (NewWConfPst ?w1 ?o ?pst ?w2))
    ;)

  ;(:function (MoveCost ?t)
  ;  (and (BTraj ?t))
  ;)

  ;(:predicate (TrajPoseCollision ?t ?o2 ?p2)
  ;  (and (BTraj ?t) (Pose ?o2 ?p2))
  ;)
  ;(:predicate (TrajArmCollision ?t ?a ?q)
  ;  (and (BTraj ?t) (AConf ?a ?q))
  ;)
  ;(:predicate (TrajGraspCollision ?t ?a ?o ?g)
  ;  (and (BTraj ?t) (Arm ?a) (Grasp ?o ?g))
  ;)
)
