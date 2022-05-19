(define (domain pr2-tamp)
  (:requirements :strips :equality)
  (:predicates
    (Arm ?a)
    (Sink ?o)
    (Stove ?o)
    (Drawer ?o) ;;
    (Door ?o) ;;
    (Joint ?o)

    (AConf ?a ?q)
    (BConf ?q)
    (Pose ?o ?p)
    (LinkPose ?o ?p)
    (Position ?o ?p)  ;; joint position of a body
    (Grasp ?o ?g)
    (HandleGrasp ?o ?g)

    (Controllable ?o)
    (Graspable ?o)
    (Stackable ?o ?r)
    (Containable ?o ?r)  ;;

    (Kin ?a ?o ?p ?g ?q ?t)
    (KinGraspHandle ?a ?o ?p ?g ?q ?t)  ;; grasp a handle
    (KinPullHandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?t)  ;; pull the handle
    (KinPullHandleOpen ?a ?o ?g ?q1 ?q2 ?t)  ;; pull the handle
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

    (AtPose ?o ?p)
    (AtLinkPose ?o ?p)
    (AtPosition ?o ?p)  ;; joint position of a body
    (OpenPosition ?o ?p)  ;; joint position of a body
    (ClosedPosition ?o ?p)  ;; joint position of a body
    (AtGrasp ?a ?o ?g)
    (AtHandleGrasp ?a ?o ?g)  ;; holding the handle
    (HandleGrasped ?a ?o)  ;; holding the handle
    (HandEmpty ?a)
    (AtBConf ?q)
    (AtAConf ?a ?q)
    (CanMove)
    (Cleaned ?o)
    (Cooked ?o)
    (OpenedDrawer ?o) ;;
    (OpenedDoor ?o) ;;
    (Opened ?o) ;;
    (OpenedJoint ?o) ;;
    (ClosedJoint ?o) ;;

    (On ?o ?r)
    (In ?o ?r) ;;
    (Reachable ?o ?r) ;;
    (Holding ?a ?o)

    (UnsafePose ?o ?p)
    (UnsafeApproach ?o ?p ?g)
    (UnsafeATraj ?t)
    (UnsafeBTraj ?t)
    (PoseObstacle ?o ?p ?o2)
    (ApproachObstacle ?o ?p ?g ?o2)
    (ATrajObstacle ?t ?o)
  )
  (:functions
    (MoveCost ?t)
    (PickCost)
    (PlaceCost)
  )

  (:action move_base
    :parameters (?q1 ?q2 ?t)
    :precondition (and (BaseMotion ?q1 ?t ?q2)
                       (AtBConf ?q1) (CanMove)
                       (not (UnsafeBTraj ?t)))
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

  (:action pick--no-atbconf
    :parameters (?a ?o ?p ?g ?q ?t)
    :precondition (and (Kin ?a ?o ?p ?g ?q ?t)
                       (AtPose ?o ?p) (HandEmpty ?a) ;(AtBConf ?q)
                       (not (UnsafeApproach ?o ?p ?g))
                       (not (UnsafeATraj ?t)))
    :effect (and (AtGrasp ?a ?o ?g) (CanMove)
                 ;(forall (?r) (when (Supported ?o ?p ?r) (not (On ?o ?r))))
                 (not (AtPose ?o ?p)) (not (HandEmpty ?a))
                 (increase (total-cost) (PickCost)))
  )
  (:action place--no-atbconf
    :parameters (?a ?o ?p ?g ?q ?t)
    :precondition (and (Kin ?a ?o ?p ?g ?q ?t)
                       (AtGrasp ?a ?o ?g) ;(AtBConf ?q)
                       (not (UnsafePose ?o ?p))
                       (not (UnsafeApproach ?o ?p ?g))
                       (not (UnsafeATraj ?t))
                       )
    :effect (and (AtPose ?o ?p) (HandEmpty ?a) (CanMove)
                 (not (AtGrasp ?a ?o ?g))
                 ;(forall (?r) (when (Supported ?o ?p ?r) (On ?o ?r)))
                 (increase (total-cost) (PlaceCost)))
  )

  (:action clean
    :parameters (?o ?r)
    :precondition (and (Stackable ?o ?r) (Sink ?r)
                       (On ?o ?r))
    :effect (Cleaned ?o)
  )
  (:action cook
    :parameters (?o ?r)
    :precondition (and (Stackable ?o ?r) (Stove ?r)
                       (On ?o ?r) (Cleaned ?o))
    :effect (and (Cooked ?o)
                 (not (Cleaned ?o)))
  )

    (:action grasp_handle
      :parameters (?a ?o ?p ?g ?q ?t)
      :precondition (and (Joint ?o)
                         (KinGraspHandle ?a ?o ?p ?g ?q ?t)
                         (AtLinkPose ?o ?p) (HandEmpty ?a) (AtBConf ?q)
                    )
      :effect (and (AtHandleGrasp ?a ?o ?g) (CanMove)
                   (not (HandEmpty ?a))
                   (increase (total-cost) (PickCost))
              )
    )

    ;; from fully closed position ?p1 pull to the fully open position ?p2
    (:action pull_handle
      :parameters (?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?t)
      :precondition (and (Joint ?o) (AtPosition ?o ?p1) (Position ?o ?p2)
                         (KinPullHandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?t)
                         (AtHandleGrasp ?a ?o ?g) (AtBConf ?q1)
                         ;(not (UnsafeApproach ?o ?p ?g))
                         ;(not (UnsafeATraj ?t))
                    )
      :effect (and (Opened ?o)
                  (AtPosition ?o ?p2) (not (AtPosition ?o ?p1))
                  (AtBConf ?q2) (not (AtBConf ?q1))
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
    (exists (?p) (and (Joint ?o) (Position ?o ?p) (AtPosition ?o ?p)
                      (OpenPosition ?o ?p)))
  )
  (:derived (ClosedJoint ?o)
    (exists (?p) (and (Joint ?o) (Position ?o ?p) (AtPosition ?o ?p)
                      (ClosedPosition ?o ?p)))
  )

    (:derived (HandleGrasped ?a ?o)
      (exists (?g) (and ;;(Arm ?a) (Joint ?o) ;;(HandleGrasp ?o ?g)
                        (AtHandleGrasp ?a ?o ?g)))
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
