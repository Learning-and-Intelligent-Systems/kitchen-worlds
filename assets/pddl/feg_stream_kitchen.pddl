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
    :inputs (?a ?o ?p ?g)
    :domain (and (Controllable ?a) (Pose ?o ?p) (Grasp ?o ?g))
    :outputs (?q ?t)
    :certified (and (SEConf ?q) (Traj ?t) (Kin ?a ?o ?p ?g ?q ?t))
  )
  (:stream plan-free-motion-hand
    :inputs (?q1 ?q2)
    :domain (and (SEConf ?q1) (SEConf ?q2))
    :fluents (AtPose AtGrasp AtSEConf AtPosition)
    :outputs (?t)
    :certified (and (Traj ?t) (FreeMotion ?q1 ?t ?q2))
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

  ;(:stream test-cfree-traj-pose
  ;  :inputs (?t ?o2 ?p2)
  ;  :domain (and (Traj ?t) (Pose ?o2 ?p2))
  ;  :certified (CFreeTrajPose ?t ?o2 ?p2)
  ;)

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
      :inputs (?a ?o ?p ?g)
      :domain (and (Controllable ?a) (Position ?o ?p) (HandleGrasp ?o ?g))
      :outputs (?q1 ?t)
      :certified (and (SEConf ?q1) (Traj ?t) (KinGraspHandle ?a ?o ?p ?g ?q1 ?t) (KinHandle ?a ?o ?p ?g ?q1))
    )
    (:stream plan-grasp-pull-handle
      :inputs (?a ?o ?pst1 ?pst2 ?g ?q1 ?q2)
      :domain (and (Controllable ?a) (Joint ?o) (Position ?o ?pst1) (Position ?o ?pst2)
                   (HandleGrasp ?o ?g) (SEConf ?q1) (SEConf ?q2)
                   (KinHandle ?a ?o ?pst1 ?g ?q1) (KinHandle ?a ?o ?pst2 ?g ?q2)
                   (IsClosedPosition ?o ?pst1) (IsOpenedPosition ?o ?pst2))
      :outputs (?t)
      :certified (and (Traj ?t) (KinPullDoorHandle ?a ?o ?pst1 ?pst2 ?g ?q1 ?q2 ?t))
    )

  ;; -------- already put those possible NewPoseFromAttachment in init -----------
  ;(:stream get-pose-from-attachment
  ;  :inputs (?o)
  ;  :domain (and (Graspable ?o))
  ;  :outputs (?p)
  ;  :certified (and (Pose ?o ?p) (NewPoseFromAttachment ?o ?p))
  ;)
  ;; ----------------------------------------------------------

  ;(:function (MoveCost ?t)
  ;  (and (Traj ?t))
  ;)
)
