(define
  (problem block_kitchen)
  (:domain block_kitchen_domain)

  (:objects
    left
    right
  )

  (:init
    ;; discrete facts (e.g. types, affordances)
    (canmove)

    (arm left)
    (arm right)

    (controllable left)
    (controllable right)
    
    (handempty left)
    (handempty right)

    (floor floor)

    (graspable steak)
    (graspable cabbage)
    (graspable salt)
    (graspable pepper)
    (graspable m2)
    (graspable m7)
    (graspable m8)
    (graspable m9)
    ;(graspable m10)
    ;(graspable m11)
    ;(graspable m12)

    (bconf q=(-0.6, 0.0, 0.0))
    (atbconf q=(-0.6, 0.0, 0.0))

    (surface table)
    (surface sink)
    (surface stove)

    (workspace lo=(-1.25, -1.25, 0.0) hi=(1.25, 1.25, 2.0))

 )
  ; Tidy goal
  ; (:goal (and (on cabbage stove) (on pepper stove) (on salt stove) (on pepper stove)))
  ; Stacking a tower
  ; (:goal (and (on m7 m8) (on m2 m7) (on steak m2) (on pepper steak)))
  ; Tricky stacking
  ; (:goal (and (on pepper salt) (on salt sink)))
  (:goal (and (on m9 stove)))
)

