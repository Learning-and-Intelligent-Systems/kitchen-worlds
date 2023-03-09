(define
  (problem rgb_triplet)
  (:domain rgb_triplet_domain)

  (:objects
    left
    right
  )

  (:init
    ;; discrete facts (e.g. types, affordances)
    ;; (canmove)

    ;(arm left)
    (arm right)

    ;(controllable left)
    (controllable right)
    
    ;(handempty left)
    (handempty right)

    (floor floor#1_1)

    (graspable g6_sds4_shr0)
    (graspable r3_sds4_shr75)
    (graspable r5_sds4_shr0)

    (bconf q=(-0.75, 0.0, 0.0))  ; was -0.6
    (atbconf q=(-0.75, 0.0, 0.0))

    (surface pillar#3_1)
    (surface pillar#4_1)
    (surface table#1_1)

    (atpose floor#1_1 p0=(0.0, 0.0, -0.1, 0))

    ;(atpose g6_sds4_shr0 p0=(0.02531642398271417, -0.011618080752010158, 0.8615000181198522, 1.8376001513223945))

    (workspace lo=(-2.0, -3., 0.0) hi=(1.0, 3., 2.0))

 )

  (:goal (and
    (on g6_sds4_shr0 r3_sds4_shr75)
    (on r5_sds4_shr0 g6_sds4_shr0)
    

  ))
)

