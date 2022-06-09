(define (stream fe-gripper-tamp)

  (:stream sample-pose-on
    :inputs (?o ?r)
    :domain (Stackable ?o ?r)
    :outputs (?p)
    :certified (and (Pose ?o ?p) (Supported ?o ?p ?r))
  )
  (:stream sample-pose-in
    :inputs (?o ?r)
    :domain (Containable ?o ?r)
    :outputs (?p)
    :certified (and (Pose ?o ?p) (Contained ?o ?p ?r))
  )
  (:stream sample-grasp
    :inputs (?o)
    :domain (Graspable ?o)
    :outputs (?g)
    :certified (Grasp ?o ?g)
  )
  (:stream inverse-kinematics-hand
    :inputs (?a ?o ?p ?g ?w)
    :domain (and (Controllable ?a) (Pose ?o ?p) (Grasp ?o ?g) (WConf ?w))
    :outputs (?q1 ?q2 ?t)
    :certified (and (SEConf ?q1) (SEConf ?q2) (Traj ?t) (KinWConf ?a ?o ?p ?g ?q1 ?q2 ?t ?w))
  )
  (:stream plan-free-motion-hand
    :inputs (?q1 ?q2 ?w)
    :domain (and (SEConf ?q1) (SEConf ?q2) (WConf ?w))
    :outputs (?t)
    :certified (and (Traj ?t) (FreeMotionWConf ?q1 ?t ?q2 ?w))
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
  (:stream test-cfree-traj-pose
    :inputs (?t ?o2 ?p2)
    :domain (and (Traj ?t) (Pose ?o2 ?p2))
    :certified (CFreeTrajPose ?t ?o2 ?p2)
  )

  (:stream get-joint-position-open
    :inputs (?o ?p1)
    :domain (and (Joint ?o) (Position ?o ?p1) (IsClosedPosition ?o ?p1))
    :outputs (?p2)
    :certified (and (Position ?o ?p2) (IsOpenedPosition ?o ?p2))
  )
    (:stream sample-handle-grasp
      :inputs (?o)
      :domain (Joint ?o)
      :outputs (?g)
      :certified (HandleGrasp ?o ?g)
    )
    (:stream inverse-kinematics-grasp-handle
      :inputs (?a ?o ?p ?g ?w)
      :domain (and (Controllable ?a) (Position ?o ?p) (HandleGrasp ?o ?g) (WConf ?w))
      :outputs (?q1 ?q2 ?t)
      :certified (and (SEConf ?q1) (SEConf ?q2) (Traj ?t) (KinGraspHandle ?a ?o ?p ?g ?q1 ?q2 ?t ?w))
    )
    (:stream plan-base-pull-door-handle
      :inputs (?a ?o ?pst1 ?pst2 ?g ?q1 ?w)
      :domain (and (Controllable ?a) (Joint ?o) (Position ?o ?pst1) (Position ?o ?pst2) (HandleGrasp ?o ?g) (SEConf ?q1) (IsClosedPosition ?o ?pst1) (IsOpenedPosition ?o ?pst2) (WConf ?w))
      :outputs (?q2 ?t)
      :certified (and (SEConf ?q2) (Traj ?t) (KinPullDoorHandle ?a ?o ?pst1 ?pst2 ?g ?q1 ?q2 ?t ?w))
    )

    ;; -------- already put those possible w2 in init -----------
    ;(:stream update-wconf-pst
    ;  :inputs (?w1 ?o ?pst)
    ;  :domain (and (WConf ?w1) (Position ?o ?pst))
    ;  :outputs (?w2)
    ;  :certified (and (WConf ?w2) (NewWConfPst ?w1 ?o ?pst ?w2))
    ;)
    ;; ----------------------------------------------------------

  (:stream test-reachable-pose
    :inputs (?o ?p ?g ?q ?w)
    :domain (and (Pose ?o ?p) (Grasp ?o ?g) (SEConf ?q) (WConf ?w))
    :certified (ReachableMovable ?o ?p ?g ?q ?w)
  )

    ;(:stream update-wconf-pst-for-reachability
    ;  :inputs (?w1 ?o ?p ?q ?g)
    ;  :domain (and (WConf ?w1) (Pose ?o ?p) (OriginalSEConf ?q) (Grasp ?o ?g))
    ;  :outputs (?j ?pst ?w2)
    ;  :certified (and (Position ?j ?pst) (WConf ?w2) (NewWConfPst ?w1 ?o ?pst ?w2) (ReachableWConf ?o ?p ?w2))
    ;)

  ;; -------- already put those possible NewPoseFromAttachment in init -----------
  ;(:stream get-pose-from-attachment
  ;  :inputs (?o ?w)
  ;  :domain (and (Graspable ?o) (WConf ?w))
  ;  :outputs (?p)
  ;  :certified (and (Pose ?o ?p) (NewPoseFromAttachment ?o ?p ?w))
  ;)
  ;; ----------------------------------------------------------

  (:function (MoveCost ?t)
    (and (Traj ?t))
  )
)
