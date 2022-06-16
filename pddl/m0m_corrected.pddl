(define
  (problem m0m_corrected)
  (:domain m0m_corrected_domain)

  (:objects
    left
    right
  )

  (:init
    ;; discrete facts (e.g. types, affordances)
    ;; (canmove)

    (arm left)
    (arm right)

    (controllable left)
    (controllable right)
    
    (handempty left)
    (handempty right)

    (floor floor#1_1)

    (graspable banana#12_1)
    (graspable sugar_box#13_1)

    (bconf q=(-0.6, 0.0, 0.0))
    (atbconf q=(-0.6, 0.0, 0.0))

    (surface pillar#3_1)
    (surface pillar#4_1)
    (surface table#1_1)

    (workspace lo=(-2.0, -3., 0.0) hi=(1.0, 3., 2.0))

 )

  (:goal (and
    (on banana#12_1 pillar#3_1)
    (on sugar_box#13_1 pillar#3_1)
  ))
)

