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

    (ReachableMovable ?o ?p ?g ?q ?w)
    (ReachablePose ?o ?p ?w)
    (AtReachablePose ?o ?p)
    (Toggled ?o)
    (OriginalSEConf ?q)

    (Kin ?a ?o ?p ?g ?q ?t)
    (KinWConf ?a ?o ?p ?g ?q1 ?q2 ?t ?w)
    (KinGraspHandle ?a ?o ?p ?g ?q1 ?q2 ?t ?w)
    (KinPullDoorHandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?t ?w1)

    (FreeMotionWConf ?p1 ?t ?p2 ?w)
    (FreeMotion ?p1 ?t ?p2)
    (Supported ?o ?p ?r)
    (Contained ?o ?p ?s) ;; aabb contains
    (Traj ?t)

    (TrajPoseCollision ?t ?o ?p)
    (CFreePosePose ?o ?p ?o2 ?p2)
    (CFreeApproachPose ?o ?p ?g ?o2 ?p2)

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

    (Debug1)
    (Debug2)
    (Debug3)

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

  (:action pick_hand
    :parameters (?a ?o ?p ?g ?q1 ?q2 ?t ?w)
    :precondition (and (KinWConf ?a ?o ?p ?g ?q1 ?q2 ?t ?w) (HandEmpty ?a)
                       (AtPose ?o ?p) (AtSEConf ?q1)
                       (ReachablePose ?o ?p ?w) (InWConf ?w)
                       (not (UnsafeApproach ?o ?p ?g))
                       )
    :effect (and (AtGrasp ?a ?o ?g) (CanMove)
                 (not (AtPose ?o ?p)) (not (HandEmpty ?a))
                 (increase (total-cost) (PickCost)))
  )

  (:action place_hand
    :parameters (?a ?o ?p ?g ?q1 ?q2 ?t ?w)
    :precondition (and (KinWConf ?a ?o ?p ?g ?q1 ?q2 ?t ?w)
                       (AtGrasp ?a ?o ?g) (AtSEConf ?q1)
                       (ReachablePose ?o ?p ?w) (InWConf ?w)
                       (not (UnsafePose ?o ?p))
                       (not (UnsafeApproach ?o ?p ?g))
                       )
    :effect (and (AtPose ?o ?p) (CanMove) (HandEmpty ?a)
                 (not (AtGrasp ?a ?o ?g))
                 (increase (total-cost) (PlaceCost)))
  )

    (:action grasp_handle_hand
      :parameters (?a ?o ?pstn ?g ?q1 ?q2 ?t ?w)
      :precondition (and (Joint ?o) (HandEmpty ?a) (AtSEConf ?q1)
                         (KinGraspHandle ?a ?o ?pstn ?g ?q1 ?q2 ?t ?w)
                         (AtPosition ?o ?pstn) (AtSEConf ?q1) (InWConf ?w)
                    )
      :effect (and (AtHandleGrasp ?a ?o ?g) (not (HandEmpty ?a)) (Debug1)
                   (not (AtSEConf ?q1))
                   (not (CanMove)) (CanPull) (not (CanUngrasp))
                   (increase (total-cost) (PickCost))
              )
    )
    (:action ungrasp_handle_hand
      :parameters (?a ?o ?pstn ?g ?q1 ?q2 ?t ?w)
      :precondition (and (Joint ?o) (CanUngrasp)
                         (KinGraspHandle ?a ?o ?pstn ?g ?q1 ?q2 ?t ?w)
                         (AtHandleGrasp ?a ?o ?g) (AtPosition ?o ?pstn) (InWConf ?w)
                    )
      :effect (and (GraspedHandle ?o) (HandEmpty ?a) (CanMove) (Debug3)
                   (AtSEConf ?q1) (not (AtHandleGrasp ?a ?o ?g))
                   (increase (total-cost) (PlaceCost))
              )
    )

    ;; from fully closed position ?p1 pull to the fully open position ?p2
    (:action pull_door_handle_wconf
      :parameters (?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?t ?w1 ?w2)
      :precondition (and (Door ?o) (not (= ?p1 ?p2)) (CanPull)
                         (AtPosition ?o ?p1) (Position ?o ?p2) (AtHandleGrasp ?a ?o ?g)
                         (KinPullDoorHandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?t ?w1)
                         ; (AtSEConf ?q1)
                         (InWConf ?w1) (WConf ?w2)
                         (NewWConfPst ?w1 ?o ?p2 ?w2)
                         ;(not (UnsafeApproach ?o ?p2 ?g))
                         ;(not (UnsafeATraj ?at))
                         ;(not (UnsafeBTraj ?bt))
                    )
      :effect (and (not (CanPull)) (CanUngrasp) (Debug2)
                  (AtPosition ?o ?p2) (not (AtPosition ?o ?p1))
                  ; (AtSEConf ?q2) (not (AtSEConf ?q1))
                  (InWConf ?w2) (not (InWConf ?w1))
              )
    )

  ;(:action toggle
  ;  :parameters (?o ?p1 ?p2 ?w1 ?w2)
  ;  :precondition (and (Joint ?o) (not (= ?p1 ?p2))
  ;                     (AtPosition ?o ?p1) (Position ?o ?p2)
  ;                     (InWConf ?w1) (NewWConfPst ?w1 ?o ?p2 ?w2)
  ;                )
  ;  :effect (and (InWConf ?w2) (not (InWConf ?w1)) (Toggled ?o)
  ;               (AtPosition ?o ?p2) (not (AtPosition ?o ?p1))
  ;          )
  ;)

  (:derived (On ?o ?r)
    (exists (?p) (and (Supported ?o ?p ?r) (AtPose ?o ?p)))
  )
  (:derived (In ?o ?r)
    (exists (?p) (and (Contained ?o ?p ?r) (AtPose ?o ?p)))
  )
  (:derived (Holding ?a ?o)
    (exists (?g) (and (Grasp ?o ?g) (AtGrasp ?a ?o ?g)))
  )
  (:derived (ReachablePose ?o ?p ?w)
    (exists (?q ?g) (and (Pose ?o ?p) (OriginalSEConf ?q) (Grasp ?o ?g) (WConf ?w) (ReachableMovable ?o ?p ?g ?q ?w)))
  )
  (:derived (AtReachablePose ?o ?p)
    (exists (?w) (and (InWConf ?w) (ReachablePose ?o ?p ?w)))
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
)
