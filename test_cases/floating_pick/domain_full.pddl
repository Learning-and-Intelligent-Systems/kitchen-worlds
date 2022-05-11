(define (domain fe-gripper-tamp)
  (:requirements :strips :equality)
  (:predicates

    (Drawer ?o) ;;
    (Door ?o) ;;
    (Knob ?o) ;;
    (Joint ?o)

    (Edible ?o)
    (CleaningSurface ?s)
    (HeatingSurface ?s)
    (ControlledBy ?s ?n)

    (HandEmpty ?a)
    (SEConf ?q)
    (Pose ?o ?p)
    (Position ?o ?p)  ;; joint position of a body
    (IsOpenedPosition ?o ?p)  ;;
    (IsClosedPosition ?o ?p)  ;; assume things start out closed
    (Grasp ?o ?g)
    (HandleGrasp ?o ?g)

    (WConf ?w)
    (InWConf ?w)
    (NewWConfP ?w1 ?o ?p ?w2)
    (NewWConfPst ?w1 ?o ?pst ?w2)

    (Graspable ?o)
    (Stackable ?o ?r)
    (Containable ?o ?r)  ;;

    (Kin ?a ?o ?p ?g ?q ?t)
    (KinWConf ?a ?o ?p ?g ?q ?t ?w)
    (KinGraspHandle ?a ?o ?p ?g ?q ?t ?w)
    (KinPullDoorHandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?t ?w1)

    (FreeMotionWConf ?p1 ?t ?p2 ?w)
    (FreeMotion ?p1 ?t ?p2)
    (Supported ?o ?p ?r)
    (Contained ?o ?p ?s) ;; aabb contains
    (Traj ?t)

    (TrajPoseCollision ?t ?o ?p)
    (CFreePosePose ?o ?p ?o2 ?p2)
    (CFreeApproachPose ?o ?p ?g ?o2 ?p2)
    (CFreeTrajPose ?t ?o2 ?p2)

    (AtSEConf ?q)
    (AtPose ?o ?p)
    (AtPosition ?o ?p)  ;; joint position of a body
    (OpenPosition ?o ?p)  ;; joint position of a body
    (ClosedPosition ?o ?p)  ;; joint position of a body

    (AtGrasp ?a ?o ?g)
    (AtHandleGrasp ?a ?o ?g)  ;; holding the handle
    (HandleGrasped ?a ?o)  ;; holding the handle
    (KnobTurned ?o)  ;; holding the knob

    (CanMove)
    (CanPull)
    (CanUngrasp)
    (Cleaned ?o)
    (Cooked ?o)
    (OpenedJoint ?o) ;;
    (ClosedJoint ?o) ;;
    (GraspedHandle ?o) ;;

    (On ?o ?r)
    (In ?o ?r) ;;
    (Holding ?a ?o)

    (UnsafePose ?o ?p)
    (UnsafeApproach ?o ?p ?g)
    (UnsafeTraj ?t)

    (PoseObstacle ?o ?p ?o2)
    (ApproachObstacle ?o ?p ?g ?o2)

  )
  (:functions
    (MoveCost ?t)
    (PickCost)
    (PlaceCost)
  )

  (:action move_cartesian
    :parameters (?q1 ?q2 ?t ?w)
    :precondition (and (CanMove) (AtSEConf ?q1) (InWConf ?w)
                       (FreeMotionWConf ?q1 ?t ?q2 ?w)
                       (not (UnsafeTraj ?t))
                   )
    :effect (and (AtSEConf ?q2) (not (AtSEConf ?q1)) (not (CanMove))
                 (increase (total-cost) (MoveCost ?t)))
  )

  (:action pick
    :parameters (?a ?o ?p ?g ?q ?t ?w)
    :precondition (and (KinWConf ?a ?o ?p ?g ?q ?t ?w) (HandEmpty ?a)
                       (AtPose ?o ?p) (AtSEConf ?q) (InWConf ?w)
                       ;(not (UnsafeApproach ?o ?p ?g))
                       ;(not (UnsafeTraj ?t))
                       )
    :effect (and (AtGrasp ?a ?o ?g) (CanMove)
                 (not (AtPose ?o ?p)) (not (HandEmpty ?a))
                 (increase (total-cost) (PickCost)))
  )

  (:action place
    :parameters (?a ?o ?p ?g ?q ?t ?w)
    :precondition (and (KinWConf ?a ?o ?p ?g ?q ?t ?w)
                       (AtGrasp ?a ?o ?g) (AtSEConf ?q) (InWConf ?w)
                       ;(not (UnsafePose ?o ?p))
                       ;(not (UnsafeApproach ?o ?p ?g))
                       ;(not (UnsafeTraj ?t))
                       )
    :effect (and (AtPose ?o ?p) (CanMove) (HandEmpty ?a)
                 (not (AtGrasp ?a ?o ?g))
                 (increase (total-cost) (PlaceCost)))
  )

    (:action grasp_handle_wconf
      :parameters (?a ?o ?p ?g ?q ?t ?w)
      :precondition (and (Joint ?o) (HandEmpty ?a)
                         (KinGraspHandle ?a ?o ?p ?g ?q ?t ?w)
                         (AtPosition ?o ?p) (AtSEConf ?q) (InWConf ?w)
                    )
      :effect (and (AtHandleGrasp ?a ?o ?g) (not (HandEmpty ?a))
                   (not (CanMove)) (CanPull) (not (CanUngrasp))
                   (increase (total-cost) (PickCost))
              )
    )

    ;; from fully closed position ?p1 pull to the fully open position ?p2
    (:action pull_door_handle_wconf
      :parameters (?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?t ?w1 ?w2)
      :precondition (and (Door ?o) (not (= ?p1 ?p2)) (CanPull)
                         (AtPosition ?o ?p1) (Position ?o ?p2) (AtHandleGrasp ?a ?o ?g)
                         (KinPullDoorHandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?t ?w1)
                         (AtSEConf ?q1) (InWConf ?w1) (WConf ?w2)
                         (NewWConfPst ?w1 ?o ?p2 ?w2)
                         ;(not (UnsafeApproach ?o ?p2 ?g))
                         ;(not (UnsafeATraj ?at))
                         ;(not (UnsafeBTraj ?bt))
                    )
      :effect (and (not (CanPull)) (CanUngrasp)
                  (AtPosition ?o ?p2) (not (AtPosition ?o ?p1))
                  (AtSEConf ?q2) (not (AtSEConf ?q1))
                  (InWConf ?w2) (not (InWConf ?w1))
              )
    )
    (:action ungrasp_handle_wconf
      :parameters (?a ?o ?p ?g ?q ?t ?w)
      :precondition (and (Joint ?o) (CanUngrasp)
                         (KinGraspHandle ?a ?o ?p ?g ?q ?t ?w)
                         (AtHandleGrasp ?a ?o ?g) (AtPosition ?o ?p)
                         (AtSEConf ?q) (InWConf ?w)
                    )
      :effect (and (GraspedHandle ?o) (HandEmpty ?a) (CanMove)
                   (not (AtHandleGrasp ?a ?o ?g))
                   (increase (total-cost) (PlaceCost))
              )
    )

  (:derived (On ?o ?r)
    (exists (?p) (and (Supported ?o ?p ?r) (AtPose ?o ?p)))
  )
  (:derived (In ?o ?r)
    (exists (?p) (and (Contained ?o ?p ?r) (AtPose ?o ?p)))
  )
  (:derived (Holding ?a ?o)
    (exists (?g) (and (Grasp ?o ?g) (AtGrasp ?a ?o ?g)))
  )

  (:derived (OpenedJoint ?o)
    (exists (?pstn) (and (Joint ?o) (Position ?o ?pstn) (AtPosition ?o ?pstn)
                      (IsOpenedPosition ?o ?pstn)))
  )
  (:derived (ClosedJoint ?o)
    (exists (?pstn) (and (Joint ?o) (Position ?o ?pstn) (AtPosition ?o ?pstn)
                      (IsClosedPosition ?o ?pstn)))
  )

    (:derived (HandleGrasped ?a ?o)
      (exists (?hg) (and (Joint ?o) (HandleGrasp ?o ?hg)
                        (AtHandleGrasp ?a ?o ?hg)))
    )

  (:derived (UnsafePose ?o ?p)
    (exists (?o2 ?p2) (and (Pose ?o ?p) (Pose ?o2 ?p2) (not (= ?o ?o2))
                           (not (CFreePosePose ?o ?p ?o2 ?p2))
                           (AtPose ?o2 ?p2)))
  )
  (:derived (UnsafeApproach ?o ?p ?g)
    (exists (?o2 ?p2) (and (Pose ?o ?p) (Grasp ?o ?g) (Pose ?o2 ?p2) (not (= ?o ?o2))
                           (not (CFreeApproachPose ?o ?p ?g ?o2 ?p2))
                           (AtPose ?o2 ?p2)))
  )
  (:derived (UnsafeTraj ?t)
    (exists (?o2 ?p2) (and (Traj ?t) (Pose ?o2 ?p2)
                           (not (CFreeTrajPose ?t ?o2 ?p2))
                           (AtPose ?o2 ?p2)))
  )

)
