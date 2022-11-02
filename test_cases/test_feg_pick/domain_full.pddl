(define (domain fe-gripper-tamp)
  (:requirements :strips :equality)

  (:constants
    @movable @bottle @edible
  )

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

    (Graspable ?o)
    (Stackable ?o ?r)
    (Containable ?o ?r)  ;;

    (ReachableMovable ?o ?p ?g ?q)
    (Reachable ?o ?p)
    (ReachablePose ?o ?p)
    (ReachableObject ?o ?p)
    (AtReachablePose ?o ?p)
    (Toggled ?o)
    (OriginalSEConf ?q)

    (Kin ?a ?o ?p ?g ?q1 ?q2 ?t)
    (KinGraspHandle ?a ?o ?p ?g ?q1 ?q2 ?t)
    (KinPullDoorHandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?t)

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

    (Debug1)
    (Debug2)
    (Debug3)
    (Debug4)

    (OfType ?o ?t)
    (StoredInSpace ?t ?r)
    (Space ?r)
    (ContainObj ?o)
    (AtAttachment ?o ?j)
    (NewPoseFromAttachment ?o ?p)

    (Cleaned ?o)
    (Cooked ?o)
  )

  (:functions
    (MoveCost ?t)
    (PickCost)
    (PlaceCost)
  )

  (:action move_cartesian
    :parameters (?q1 ?q2 ?t)
    :precondition (and (CanMove) (AtSEConf ?q1)
                       (FreeMotion ?q1 ?t ?q2)
                       (not (UnsafeTraj ?t))
                   )
    :effect (and (AtSEConf ?q2) (not (AtSEConf ?q1)) (not (CanMove))
                 (increase (total-cost) (MoveCost ?t)))
  )

  (:action pick_hand
    :parameters (?a ?o ?p ?g ?q1 ?q2 ?t)
    :precondition (and (Kin ?a ?o ?p ?g ?q1 ?q2 ?t) (HandEmpty ?a)
                       (ReachablePose ?o ?p) (AtSEConf ?q1)
                       (not (CanMove))
                       (AtPose ?o ?p)
                       (not (UnsafeApproach ?o ?p ?g))
                       )
    :effect (and (AtGrasp ?a ?o ?g) (CanMove)
                 (not (AtPose ?o ?p)) (not (HandEmpty ?a))
                 (increase (total-cost) (PickCost)))
  )

  (:action place_hand
    :parameters (?a ?o ?p ?g ?q1 ?q2 ?t)
    :precondition (and (Kin ?a ?o ?p ?g ?q1 ?q2 ?t)
                       (ReachablePose ?o ?p) (AtSEConf ?q1)
                       (AtGrasp ?a ?o ?g)
                       (not (CanMove))
                       (not (UnsafePose ?o ?p))
                       (not (UnsafeApproach ?o ?p ?g))
                       )
    :effect (and (AtPose ?o ?p) (CanMove) (HandEmpty ?a)
                 (not (AtGrasp ?a ?o ?g))
                 (increase (total-cost) (PlaceCost)))
  )

    (:action grasp_handle_hand
      :parameters (?a ?o ?pstn ?g ?q1 ?q2 ?t)
      :precondition (and (Joint ?o) (HandEmpty ?a) (AtSEConf ?q1)
                         (KinGraspHandle ?a ?o ?pstn ?g ?q1 ?q2 ?t)
                         (AtPosition ?o ?pstn) (AtSEConf ?q1)
                    )
      :effect (and (AtHandleGrasp ?a ?o ?g) (not (HandEmpty ?a)) (Debug1)
                   (not (AtSEConf ?q1))
                   (not (CanMove)) (CanPull) (not (CanUngrasp))
                   (increase (total-cost) (PickCost))
              )
    )
    (:action ungrasp_handle_hand
      :parameters (?a ?o ?pstn ?g ?q1 ?q2 ?t)
      :precondition (and (Joint ?o) (CanUngrasp)
                         (KinGraspHandle ?a ?o ?pstn ?g ?q1 ?q2 ?t)
                         (AtHandleGrasp ?a ?o ?g) (AtPosition ?o ?pstn)
                    )
      :effect (and (GraspedHandle ?o) (HandEmpty ?a) (CanMove)
                   (AtSEConf ?q1) (not (AtHandleGrasp ?a ?o ?g))
                   (increase (total-cost) (PlaceCost))
              )
    )

    ;; from fully closed position ?p1 pull to the fully open position ?p2
    (:action pull_articulated_handle
      :parameters (?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?t)
      :precondition (and (Joint ?o) (not (= ?p1 ?p2)) (CanPull)
                         (AtPosition ?o ?p1) (Position ?o ?p2) (AtHandleGrasp ?a ?o ?g)
                         (KinPullDoorHandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?t)
                         ; (AtSEConf ?q1)
                         ;(not (UnsafeApproach ?o ?p2 ?g))
                         ;(not (UnsafeATraj ?at))
                         ;(not (UnsafeBTraj ?bt))
                    )
      :effect (and (not (CanPull)) (CanUngrasp) (Debug2)
                  (AtPosition ?o ?p2) (not (AtPosition ?o ?p1))
                  ; (AtSEConf ?q2) (not (AtSEConf ?q1))

                  ;(forall (?o3 ?p3 ?p4) (when
                  ;  (and (ContainObj ?o3) (AtPose ?o3 ?p3) (Pose ?o3 ?p4) (AtAttachment ?o3 ?o) (NewPoseFromAttachment ?o3 ?p4))
                  ;  (and (not (AtPose ?o3 ?p3)) (AtPose ?o3 ?p4) (Debug4))
                  ;))
              )
    )

    ;; with attachment
    (:action pull_articulated_handle_attachment
      :parameters (?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?t ?o3 ?p3 ?p4)
      :precondition (and (Joint ?o) (not (= ?p1 ?p2)) (CanPull)
                         (AtPosition ?o ?p1) (Position ?o ?p2) (AtHandleGrasp ?a ?o ?g)
                         (KinPullDoorHandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?t)

                         (ContainObj ?o3) (AtPose ?o3 ?p3) (Pose ?o3 ?p4)
                         (AtAttachment ?o3 ?o) (NewPoseFromAttachment ?o3 ?p4)
                    )
      :effect (and (not (CanPull)) (CanUngrasp) (Debug2)
                  (AtPosition ?o ?p2) (not (AtPosition ?o ?p1))
                  (not (AtPose ?o3 ?p3)) (AtPose ?o3 ?p4) (Debug4)
              )
    )

    (:action declare_store_in_space
      :parameters (?t ?r)
      :precondition (and (Space ?r)
                         (forall (?o) (imply (OfType ?o ?t) (In ?o ?r)))
                    )
      :effect (and (StoredInSpace ?t ?r))
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

  ;(:action toggle
  ;  :parameters (?o ?p1 ?p2)
  ;  :precondition (and (Joint ?o) (not (= ?p1 ?p2))
  ;                     (AtPosition ?o ?p1) (Position ?o ?p2)
  ;                )
  ;  :effect (and (Toggled ?o)
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
  ;(:derived (ReachableObject ?o ?p)
  ;  (or (ReachablePose ?o ?p) (Reachable ?o ?p))
  ;)
  (:derived (ReachablePose ?o ?p)
    (exists (?q ?g) (and (Pose ?o ?p) (OriginalSEConf ?q) (Grasp ?o ?g) (ReachableMovable ?o ?p ?g ?q)))
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
    (exists (?o2 ?p2) (and (Traj ?t) (Pose ?o2 ?p2) (AtPose ?o2 ?p2)
                           (not (CFreeTrajPose ?t ?o2 ?p2))))
  )
)
