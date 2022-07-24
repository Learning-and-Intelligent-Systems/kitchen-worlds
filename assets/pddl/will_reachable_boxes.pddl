(define
  (problem will_reachable_boxes)
  (:domain will_reachable_boxes_domain)

  (:objects
    right
  )

  (:init
    ;; discrete facts (e.g. types, affordances)
    (arm right)
    (controllable right)
    (handempty right)

    (floor table)

    ;(graspable box_1)
    ;(graspable box_2)
    (graspable box_3)
    (graspable box_4)
    (graspable box_5)
    (graspable box_6)
    (graspable box_7)
    (graspable box_8)
    (graspable box_9)
    (graspable box_10)
    (graspable box_11)
    (graspable box_12)

    (surface table)
    (surface fixed_plane)

 )
  (:goal (and
  (on box_5 fixed_plane)
  ))
)

