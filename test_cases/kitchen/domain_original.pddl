(define (domain pr2-tamp)
  (:requirements :strips :equality)
  (:predicates
    (Arm ?a)
    (Sink ?o)
    (Stove ?o)
    (Drawer ?o) ;;
    (Door ?o) ;;
    (Knob ?o) ;;
    (Joint ?o)

    (Edible ?o)
    (CleaningSurface ?s)
    (HeatingSurface ?s)
    (ControlledBy ?s ?n)

    (AConf ?a ?q)
    (DefaultAConf ?a ?q)
    (BConf ?q)
    (UngraspBConf ?q)
    (Pose ?o ?p)
    (MagicPose ?o ?p)  ;; for teleport debugging
    (LinkPose ?o ?p)
    (Position ?o ?p)  ;; joint position of a body
    (IsOpenedPosition ?o ?p)  ;;
    (IsClosedPosition ?o ?p)  ;; assume things start out closed
    (Grasp ?o ?g)
    (HandleGrasp ?o ?g)

    (WConf ?w)
    (InWConf ?w)
    (NewWConfP ?w1 ?o ?p ?w2)
    (NewWConfPst ?w1 ?o ?pst ?w2)

    (Controllable ?o)
    (Graspable ?o)
    (Stackable ?o ?r)
    (Containable ?o ?r)  ;;

    (Kin ?a ?o ?p ?g ?q ?t)
    (KinWConf ?a ?o ?p ?g ?q ?t ?w)
    (KinGraspHandle ?a ?o ?p ?g ?q ?aq ?t)  ;; grasp a handle
    (KinGraspHandleWConf ?a ?o ?p ?g ?q ?aq ?t ?w)  ;; grasp a handle
    (KinUngraspHandle ?a ?o ?p ?g ?q ?aq1 ?aq2 ?t)  ;; ungrasp a handle
    (KinUngraspHandleWConf ?a ?o ?p ?g ?q ?aq1 ?aq2 ?t ?w)  ;; ungrasp a handle
    (KinPullDrawerHandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?t)  ;; pull the handle
    (KinPullDoorHandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?bt ?aq1 ?aq2 ?at)  ;; pull the handle
    (KinTurnKnob ?a ?o ?p1 ?p2 ?g ?q ?aq1 ?aq2 ?at)

    (BaseMotionWConf ?q1 ?t ?q2 ?w)
    (BaseMotion ?q1 ?t ?q2)
    (ArmMotion ?a ?q1 ?t ?q2)
    (Supported ?o ?p ?r)
    (Contained ?o ?p ?s) ;; aabb contains
    (BTraj ?t)
    (ATraj ?t)

    (TrajPoseCollision ?t ?o ?p)
    (TrajArmCollision ?t ?a ?q)
    (TrajGraspCollision ?t ?a ?o ?g)
    (CFreePosePose ?o ?p ?o2 ?p2)
    (CFreeApproachPose ?o ?p ?g ?o2 ?p2)
    (CFreeTrajPose ?t ?o2 ?p2)
    (IsClosedPosition ?o ?p)  ;;
    (IsOpenPosition ?o ?p)  ;;

    (CFreeBTrajPose ?t ?o2 ?p2)

    (AtPose ?o ?p)
    (AtLinkPose ?o ?p)
    (AtPosition ?o ?p)  ;; joint position of a body
    (OpenPosition ?o ?p)  ;; joint position of a body
    (ClosedPosition ?o ?p)  ;; joint position of a body
    (AtGrasp ?a ?o ?g)
    (AtHandleGrasp ?a ?o ?g)  ;; holding the handle
    (HandleGrasped ?a ?o)  ;; holding the handle
    (KnobTurned ?a ?o)  ;; holding the knob
    (HandEmpty ?a)
    (AtBConf ?q)
    (AtAConf ?a ?q)

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
    (UnsafeATraj ?t)
    (UnsafeBTraj ?t)
    (PoseObstacle ?o ?p ?o2)
    (ApproachObstacle ?o ?p ?g ?o2)
    (ATrajObstacle ?t ?o)

    (Debug1)
    (Debug2)
    (Debug3)
  )
  (:functions
    (MoveCost ?t)
    (PickCost)
    (PlaceCost)
  )

  ;(:action move_base
  ;  :parameters (?q1 ?q2 ?t)
  ;  :precondition (and (CanMove) (BaseMotion ?q1 ?t ?q2)
  ;                     (AtBConf ?q1)
  ;                     (not (UnsafeBTraj ?t)))
  ;  :effect (and (AtBConf ?q2)
  ;               (not (AtBConf ?q1)) (not (CanMove))
  ;               (increase (total-cost) (MoveCost ?t)))
  ;)
  (:action move_base_wconf
    :parameters (?q1 ?q2 ?t ?w)
    :precondition (and (CanMove) (BaseMotionWConf ?q1 ?t ?q2 ?w)
                       (AtBConf ?q1) (InWConf ?w))
    :effect (and (AtBConf ?q2)
                 (not (AtBConf ?q1)) (not (CanMove))
                 (increase (total-cost) (MoveCost ?t)))
  )

  ;(:action move_arm
  ;  :parameters (?q1 ?q2 ?t)
  ;  :precondition (and (ArmMotion ?a ?q1 ?t ?q2)
  ;                     (AtAConf ?a ?q1))
  ;  :effect (and (AtAConf ?a ?q2)
  ;               (not (AtAConf ?a ?q1)))
  ;)

  (:action pick
    :parameters (?a ?o ?p ?g ?q ?t ?w)
    :precondition (and (KinWConf ?a ?o ?p ?g ?q ?t ?w)
                       (AtPose ?o ?p) (HandEmpty ?a)
                       (AtBConf ?q) (InWConf ?w)
                       (not (UnsafeApproach ?o ?p ?g))
                       (not (UnsafeATraj ?t))
                       )
    :effect (and (AtGrasp ?a ?o ?g) (CanMove)
                 ;(forall (?r) (when (Supported ?o ?p ?r) (not (On ?o ?r))))
                 (not (AtPose ?o ?p)) (not (HandEmpty ?a))
                 (increase (total-cost) (PickCost)))
  )

  (:action place
    :parameters (?a ?o ?p ?g ?q ?t ?w)
    :precondition (and (KinWConf ?a ?o ?p ?g ?q ?t ?w)
                       (AtGrasp ?a ?o ?g) (AtBConf ?q) (InWConf ?w)
                       (not (UnsafePose ?o ?p))
                       (not (UnsafeApproach ?o ?p ?g))
                       (not (UnsafeATraj ?t))
                       )
    :effect (and (AtPose ?o ?p) (HandEmpty ?a) (CanMove)
                 (not (AtGrasp ?a ?o ?g))
                 ;(forall (?r) (when (Supported ?o ?p ?r) (On ?o ?r)))
                 (increase (total-cost) (PlaceCost)))
  )

  (:action teleport
    :parameters (?o ?p1 ?p2 ?w1 ?w2)
    :precondition (and (AtPose ?o ?p1) (MagicPose ?o ?p2)
                       (WConf ?w1) (WConf ?w2)
                       ;; (InWConf ?w1) (WConf ?w2)
                       ;; (NewWConfP ?w1 ?o ?p2 ?w2)
                       )
    :effect (and ;; (not (InWConf ?w1)) (InWConf ?w2) ;(Debug1)
                 (not (AtPose ?o ?p1)) (AtPose ?o ?p2)
                 (increase (total-cost) (PickCost)))
  )

  (:action clean
    :parameters (?o ?r)
    :precondition (and (Stackable ?o ?r) (Sink ?r)
                       (On ?o ?r))
    :effect (and (Cleaned ?o))
  )
  (:action cook
    :parameters (?o ?r)
    :precondition (and (Stackable ?o ?r) (Stove ?r)
                       (On ?o ?r) (Cleaned ?o))
    :effect (and (Cooked ?o)
                 (not (Cleaned ?o)))
  )
  (:action wait-clean
    :parameters (?o ?s ?n)
    :precondition (and (Edible ?o) (CleaningSurface ?s) (ControlledBy ?s ?n)
                       (On ?o ?s) (GraspedHandle ?n)
                       )
    :effect (and (Cleaned ?o))
  )
  (:action wait-cook
    :parameters (?o ?s ?n)
    :precondition (and (Edible ?o) (HeatingSurface ?s) (ControlledBy ?s ?n)
                       (On ?o ?s) (GraspedHandle ?n)
                       (Cleaned ?o)
                       )
    :effect (and (Cooked ?o)
                 (not (Cleaned ?o)))
  )

    (:action grasp_handle
      :parameters (?a ?o ?p ?g ?q ?aq1 ?aq2 ?t)
      :precondition (and (Joint ?o)
                         (KinGraspHandle ?a ?o ?p ?g ?q ?aq2 ?t)
                         (AtPosition ?o ?p) (HandEmpty ?a)
                         (AtBConf ?q) (AtAConf ?a ?aq1)
                    )
      :effect (and (AtHandleGrasp ?a ?o ?g) (not (HandEmpty ?a))
                   (not (CanMove)) (CanPull) (not (CanUngrasp))
                   (not (AtAConf ?a ?aq1)) (AtAConf ?a ?aq2)
                   (increase (total-cost) (PickCost))
              )
    )
    (:action ungrasp_handle
      :parameters (?a ?o ?p ?g ?q ?aq1 ?aq2 ?t)
      :precondition (and (Joint ?o) (AtPosition ?o ?p)
                         (KinUngraspHandle ?a ?o ?p ?g ?q ?aq1 ?aq2 ?t)
                         (AtHandleGrasp ?a ?o ?g) (CanUngrasp)
                         (AtBConf ?q) (UngraspBConf ?q) (AtAConf ?a ?aq1) ;; (DefaultAConf ?a ?aq2)
                    )
      :effect (and (GraspedHandle ?o) (HandEmpty ?a) (CanMove)
                   (not (AtHandleGrasp ?a ?o ?g))
                   (not (AtAConf ?a ?aq1)) (AtAConf ?a ?aq2)
                   (increase (total-cost) (PlaceCost))
              )
    )
    ;; from fully closed position ?p1 pull to the fully open position ?p2
    (:action pull_drawer_handle
      :parameters (?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?bt)
      :precondition (and (Drawer ?o) (not (= ?p1 ?p2)) (CanPull)
                         (AtPosition ?o ?p1) (Position ?o ?p2)
                         (KinPullDrawerHandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?bt)
                         (AtHandleGrasp ?a ?o ?g) (AtBConf ?q1)
                         ;(not (UnsafeBTraj ?bt))
                    )
      :effect (and (not (CanPull)) (CanUngrasp)
                  (AtPosition ?o ?p2) (not (AtPosition ?o ?p1))
                  (AtBConf ?q2) (not (AtBConf ?q1))
              )
    )

    ;; from fully closed position ?p1 pull to the fully open position ?p2
    (:action pull_door_handle
      :parameters (?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?bt ?aq1 ?aq2 ?at ?w1 ?w2)
      :precondition (and (Door ?o) (not (= ?p1 ?p2)) (CanPull)
                         (AtPosition ?o ?p1) (Position ?o ?p2) (AtHandleGrasp ?a ?o ?g)
                         (KinPullDoorHandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?bt ?aq1 ?aq2 ?at)
                         (AtBConf ?q1) (AtAConf ?a ?aq1)
                         (InWConf ?w1) (WConf ?w2)
                         (NewWConfPst ?w1 ?o ?p2 ?w2)
                         ;(not (UnsafeApproach ?o ?p2 ?g))
                         ;(not (UnsafeATraj ?at))
                         ;(not (UnsafeBTraj ?bt))
                    )
      :effect (and (not (CanPull)) (CanUngrasp)
                  (AtPosition ?o ?p2) (not (AtPosition ?o ?p1))
                  (AtBConf ?q2) (not (AtBConf ?q1))
                  (AtAConf ?a ?aq2) (not (AtAConf ?a ?aq1))
                  (InWConf ?w2) (not (InWConf ?w1))
              )
    )

    ;; from fully closed position ?p1 pull to the fully open position ?p2
    (:action turn_knob
      :parameters (?a ?o ?p1 ?p2 ?g ?q ?aq1 ?aq2 ?at)
      :precondition (and (Knob ?o) (not (= ?p1 ?p2)) (CanPull)
                         (AtPosition ?o ?p1) (Position ?o ?p2) (AtHandleGrasp ?a ?o ?g)
                         (KinTurnKnob ?a ?o ?p1 ?p2 ?g ?q ?aq1 ?aq2 ?at)
                         (AtBConf ?q) (AtAConf ?a ?aq1)
                         ;(not (UnsafeApproach ?o ?p2 ?g))
                         ;(not (UnsafeATraj ?at))
                         ;(not (UnsafeBTraj ?bt))
                    )
      :effect (and (not (CanPull)) (CanUngrasp) (KnobTurned ?a ?o)
                  (AtPosition ?o ?p2) (not (AtPosition ?o ?p1))
                  (UngraspBConf ?q)
                  (AtAConf ?a ?aq2) (not (AtAConf ?a ?aq1))
              )
    )

  (:derived (On ?o ?r)
    (exists (?p) (and (Supported ?o ?p ?r)
                      (AtPose ?o ?p)))
  )
  (:derived (In ?o ?r)
    (exists (?p) (and (Contained ?o ?p ?r)
                      (AtPose ?o ?p)))
  )
  (:derived (Holding ?a ?o)
    (exists (?g) (and (Arm ?a) (Grasp ?o ?g)
                      (AtGrasp ?a ?o ?g)))
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
      (exists (?hg) (and (Arm ?a) (Joint ?o) (HandleGrasp ?o ?hg)
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
  (:derived (UnsafeATraj ?t)
    (exists (?o2 ?p2) (and (ATraj ?t) (Pose ?o2 ?p2)
                           (not (CFreeTrajPose ?t ?o2 ?p2))
                           (AtPose ?o2 ?p2)))
  )

  (:derived (UnsafeBTraj ?t)
    (exists (?o2 ?p2) (and (BTraj ?t) (Pose ?o2 ?p2) (AtPose ?o2 ?p2)
                           (not (CFreeBTrajPose ?t ?o2 ?p2))))
  )

    (:derived (PoseObstacle ?o ?p ?o2)
      (exists (?p2)
         (and (Pose ?o ?p) (Pose ?o2 ?p2) (not (= ?o ?o2))
               (not (CFreePosePose ?o ?p ?o2 ?p2))
               (AtPose ?o2 ?p2)))
    )
    (:derived (ApproachObstacle ?o ?p ?g ?o2)
      (exists (?p2)
         (and (Pose ?o ?p) (Grasp ?o ?g) (Pose ?o2 ?p2) (not (= ?o ?o2))
              (not (CFreeApproachPose ?o ?p ?g ?o2 ?p2))
              (AtPose ?o2 ?p2)))
    )
    (:derived (ATrajObstacle ?t ?o2)
      (exists (?p2)
         (and (ATraj ?t) (Pose ?o2 ?p2)
              (not (CFreeTrajPose ?t ?o2 ?p2))
              (AtPose ?o2 ?p2)))
    )

  ;(:derived (UnsafeBTraj ?t) (or
  ;  (exists (?o2 ?p2) (and (TrajPoseCollision ?t ?o2 ?p2)
  ;                         (AtPose ?o2 ?p2)))
  ;  (exists (?a ?q) (and (TrajArmCollision ?t ?a ?q)
  ;                       (AtAConf ?a ?q)))
  ;  (exists (?a ?o ?g) (and (TrajGraspCollision ?t ?a ?o ?g)
  ;                          (AtGrasp ?a ?o ?g)))
  ;))
)
