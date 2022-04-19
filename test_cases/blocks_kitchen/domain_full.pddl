(define (domain pr2-tamp)
  (:requirements :strips :equality)
  (:predicates
    (Arm ?a)
    (Stackable ?o ?r)

    (Sink ?r)
    (Stove ?r)
    (Counter ?r)
    (Table ?r)
    (Salter ?o)
    (Egg ?o)
    (Veggie ?o)
    (Plate ?o)
    (Supporter ?o)
    (Environment ?o)
    (Cart ?o)
    (Marker ?o)
    (Marked ?o ?o2)

    (AConf ?a ?q)
    (BConf ?q)
    (BConfInRegion ?q ?r)
    (PoseInRegion ?o ?p ?r)
    (InRoom ?o ?r)
    (RobInRoom ?r)

    (Pose ?o ?p)
    (Grasp ?o ?g)
    (MarkerGrasp ?o ?g)
    (Kin ?a ?o ?p ?g ?q ?t)
    (KinGraspMarker ?a ?o ?p ?g ?q ?t) ;;
    (KinUngraspMarker ?a ?o ?p ?g ?q ?t) ;;
    (KinPullMarkerRandom ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?o2 ?p3 ?p4 ?t) ;;
    (KinPullMarkerToPose ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?o2 ?p3 ?p4 ?t) ;;
    (KinPullMarkerToBConf ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?o2 ?p3 ?p4 ?t) ;;

    (BaseMotion ?q1 ?t ?q2)
    (ArmMotion ?a ?q1 ?t ?q2)
    (Supported ?o ?p ?r)
    (BTraj ?t)
    (ATraj ?t)

    (Controllable ?o)
    (Graspable ?o)
    (Stackable ?o ?r)

    (TrajPoseCollision ?t ?o ?p)
    (TrajArmCollision ?t ?a ?q)
    (TrajGraspCollision ?t ?a ?o ?g)
    (CFreePosePose ?o ?p ?o2 ?p2)
    (CFreeApproachPose ?o ?p ?g ?o2 ?p2)
    (CFreeTrajPose ?t ?o2 ?p2)
    (CFreeBTrajPose ?t ?o2 ?p2)
    ;;(CFreeBTrajWithAttachmentPose ?t ?o ?o2 ?p2) ;;

    (AtPose ?o ?p)
    (AtGrasp ?a ?o ?g)
    (AtMarkerGrasp ?a ?o ?g) ;;
    (HandEmpty ?a)
    (AtBConf ?q)
    (AtAConf ?a ?q)
    (CanMove)
    (CanPull) ;;
    (CanUngrasp) ;;

    (Cleaned ?o)
    (Cooked ?o)
    (Seasoned ?o)
    (Served ?o ?o2)
    (EnableOmelette ?egg1 ?veggie1 ?plate1)
    (ExistOmelette ?env1)

    (On ?o ?r)
    (Holding ?a ?o)
    (HoldingMarker ?a ?o) ;;
    (PulledMarker ?o) ;;
    (GraspedMarker ?o) ;;
    (SavedMarker ?o) ;;

    (UnsafePose ?o ?p)
    (UnsafeApproach ?o ?p ?g)
    (UnsafeATraj ?t)
    (UnsafeBTraj ?t)
    (PoseObstacle ?o ?p ?o2)
    (ApproachObstacle ?o ?p ?g ?o2)
    (ATrajObstacle ?t ?o)
    ;; (UnsafeBTrajWithAttachment ?t ?o) ;;
  )
  (:functions
    (MoveCost ?t)
    (PickCost)
    (PlaceCost)
  )

  (:action move_base
    :parameters (?q1 ?q2 ?t)
    :precondition (and (BaseMotion ?q1 ?t ?q2) (CanMove)
                       (AtBConf ?q1)
                       ;; (not (UnsafeBTraj ?t))
                       )
    :effect (and (AtBConf ?q2)
                 (not (AtBConf ?q1)) (not (CanMove))
                 (increase (total-cost) (MoveCost ?t)))
  )

  (:action pick--no-atbconf
    :parameters (?a ?o ?p ?g ?q ?t)
    :precondition (and (Kin ?a ?o ?p ?g ?q ?t) (not (Marker ?o))
                       (AtPose ?o ?p) (HandEmpty ?a) ;; (AtBConf ?q)
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
                       (AtGrasp ?a ?o ?g) ;; (AtBConf ?q)
                       (not (UnsafePose ?o ?p))
                       (not (UnsafeApproach ?o ?p ?g))
                       (not (UnsafeATraj ?t)))
    :effect (and (AtPose ?o ?p) (HandEmpty ?a) (CanMove)
                 (not (AtGrasp ?a ?o ?g))
                 ;(forall (?r) (when (Supported ?o ?p ?r) (On ?o ?r)))
                 (increase (total-cost) (PlaceCost)))
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
    :effect (and (Cooked ?o))
  )
  (:action season
    :parameters (?o ?r ?o2)
    :precondition (and (Stackable ?o ?r) (Counter ?r)
                       (On ?o ?r) (Cooked ?o)
                       (Stackable ?o2 ?r) (Salter ?o2)
                       (On ?o2 ?r))
    :effect (and (Seasoned ?o))
  )
  (:action serve
    :parameters (?o ?r ?o2)
    :precondition (and (Stackable ?o ?r) (Table ?r)
                       (On ?o ?r) (Seasoned ?o)
                       (Stackable ?o2 ?r) (Plate ?o2)
                       (On ?o2 ?r))
    :effect (and (Served ?o ?o2))
  )

    (:action grasp_marker
      :parameters (?a ?o ?o2 ?p ?g ?q ?t)
      :precondition (and (Cart ?o) (Marker ?o2) (Marked ?o ?o2)
                         (KinGraspMarker ?a ?o2 ?p ?g ?q ?t)
                         (AtPose ?o2 ?p) (HandEmpty ?a) (AtBConf ?q)
                    )
      :effect (and (AtMarkerGrasp ?a ?o ?g)
                   (AtMarkerGrasp ?a ?o2 ?g)
                   (not (HandEmpty ?a)) (not (CanMove))
                   (not (CanUngrasp)) ;;
                   (increase (total-cost) (PickCost))
              )
    )
  (:action ungrasp_marker
    :parameters (?a ?o ?o2 ?p ?g ?q ?t)
    :precondition (and (Cart ?o) (Marker ?o2) (Marked ?o ?o2) (AtPose ?o2 ?p)
                       (CanUngrasp) ;;
                       (KinUngraspMarker ?a ?o2 ?p ?g ?q ?t)
                       (AtMarkerGrasp ?a ?o ?g)
                       (AtMarkerGrasp ?a ?o2 ?g) (AtBConf ?q))
    :effect (and (HandEmpty ?a) (CanMove)
                 (not (AtMarkerGrasp ?a ?o ?g))
                 (not (AtMarkerGrasp ?a ?o2 ?g))
                 (GraspedMarker ?o2) ;;
                 (increase (total-cost) (PlaceCost)))
  )

    ;; marker from p1 to a random base position p2, cart moves from p3 to p4
    (:action pull_marker_random
      :parameters (?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?o2 ?p3 ?p4 ?t)
      :precondition (and (not (CanMove)) (CanPull)
                         (Marker ?o) (Cart ?o2) (Marked ?o2 ?o)
                         (KinPullMarkerRandom ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?o2 ?p3 ?p4 ?t)
                         (AtPose ?o ?p1) (AtPose ?o2 ?p3) (AtBConf ?q1)
                                             (AtMarkerGrasp ?a ?o ?g)
                         ;(not (UnsafeBTrajWithMarker ?t ?o))
                    )
      :effect (and (not (AtPose ?o ?p1)) (AtPose ?o ?p2) (PulledMarker ?o)
                   (not (AtPose ?o2 ?p3)) (AtPose ?o2 ?p4)
                   (AtBConf ?q2) (not (AtBConf ?q1))
                   (not (CanPull)) (CanUngrasp) ;;
                   (increase (total-cost) (MoveCost ?t))
              )
    )

    ;; to a sampled base position
    (:action pull_marker_to_pose
      :parameters (?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?o2 ?p3 ?p4 ?t)
      :precondition (and (not (CanMove)) (CanPull) (not (= ?p1 ?p2))
                         (Marker ?o) (Cart ?o2) (Marked ?o2 ?o)
                         (KinPullMarkerToPose ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?o2 ?p3 ?p4 ?t)
                         (AtPose ?o ?p1) (AtPose ?o2 ?p3) (AtBConf ?q1)
                                             (AtMarkerGrasp ?a ?o ?g)
                         ;(not (UnsafeBTrajWithMarker ?t ?o))
                    )
      :effect (and (not (AtPose ?o ?p1)) (AtPose ?o ?p2) (PulledMarker ?o)
                   (not (AtPose ?o2 ?p3)) (AtPose ?o2 ?p4)
                   (AtBConf ?q2) (not (AtBConf ?q1))
                   (not (CanPull)) (CanUngrasp) ;;
                   (increase (total-cost) (MoveCost ?t))
              )
    )

    ;; to a sampled object pose
    (:action pull_marker_to_bconf
      :parameters (?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?o2 ?p3 ?p4 ?t)
      :precondition (and (not (CanMove)) (CanPull) (not (= ?q1 ?q2))
                         (Marker ?o) (Cart ?o2) (Marked ?o2 ?o)
                         (KinPullMarkerToBConf ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?o2 ?p3 ?p4 ?t)
                         (AtPose ?o ?p1) (AtPose ?o2 ?p3) (AtBConf ?q1)
                                             (AtMarkerGrasp ?a ?o ?g)
                         ;(not (UnsafeBTrajWithMarker ?t ?o))
                    )
      :effect (and (not (AtPose ?o ?p1)) (AtPose ?o ?p2) (PulledMarker ?o)
                   (not (AtPose ?o2 ?p3)) (AtPose ?o2 ?p4)
                   (AtBConf ?q2) (not (AtBConf ?q1))
                   (not (CanPull)) (CanUngrasp) ;;
                   (increase (total-cost) (MoveCost ?t))
              )
    )


  (:action magic
    :parameters (?o ?o2 ?p1 ?p3)
    :precondition (and (Marker ?o) (Cart ?o2) (Marked ?o2 ?o)
                       (AtPose ?o ?p1) (AtPose ?o2 ?p3))
    :effect (and (not (AtPose ?o ?p1)) (not (AtPose ?o2 ?p3)))
  )

  (:derived (RobInRoom ?r)
    (exists (?q) (and (BConfInRegion ?q ?r) (AtBConf ?q)))
  )
  (:derived (InRoom ?o ?r)
    (exists (?p) (and (PoseInRegion ?o ?p ?r) (AtPose ?o ?p)))
  )
  (:derived (On ?o ?r)
    (exists (?p) (and (Supported ?o ?p ?r)
                      (AtPose ?o ?p)))
  )
  (:derived (Holding ?a ?o)
    (exists (?g) (and (Arm ?a) (Grasp ?o ?g)
                      (AtGrasp ?a ?o ?g)))
  )
  (:derived (HoldingMarker ?a ?o)
    (exists (?g) (and (Arm ?a) (Marker ?o) (MarkerGrasp ?o ?g)
                      (AtMarkerGrasp ?a ?o ?g)))
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
    (exists (?o2 ?p2) (and (ATraj ?t) (Pose ?o2 ?p2) (AtPose ?o2 ?p2)
                           (not (CFreeTrajPose ?t ?o2 ?p2))))
  )
  (:derived (UnsafeBTraj ?t)
    (exists (?o2 ?p2) (and (BTraj ?t) (Pose ?o2 ?p2) (AtPose ?o2 ?p2)
                           (not (CFreeBTrajPose ?t ?o2 ?p2))))
  )

  ;(:derived (UnsafeBTrajWithAttachment ?t ?o)
  ;  (exists (?o2 ?p2) (and (BTraj ?t) (Marker ?o) (Pose ?o2 ?p2)
  ;                         (not (CFreeBTrajWithAttachmentPose ?t ?o ?o2 ?p2))
  ;                         (AtPose ?o2 ?p2)))
  ;)

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

  ( :derived ( EnableOmelette ?egg1 ?veggie1 ?plate1 )
    ( and
        ( Egg ?egg1 )
        ( Veggie ?veggie1 )
        ( Plate ?plate1 )

        ( Cleaned ?veggie1 )
        ( Cooked ?egg1 )
        ( Cooked ?veggie1 )
        ( Seasoned ?egg1 )
        ( Seasoned ?veggie1 )
        ( Served ?egg1 ?plate1 )
        ( Served ?veggie1 ?plate1 )
    )
  )

    ( :derived ( ExistOmelette ?table1 )
        ( exists ( ?egg1 ?veggie1 ?plate1 )
            ( and
                ( Egg ?egg1 ) ( Veggie ?veggie1 ) ( Plate ?plate1 ) ( Table ?table1 )
                ( EnableOmelette ?egg1 ?veggie1 ?plate1 ) ( On ?plate1 ?table1 )
            )
        )
    )

)