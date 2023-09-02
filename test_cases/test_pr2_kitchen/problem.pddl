
(define
  (problem test_full_kitchen_230121_151600_original_3)
  (:domain domain)

  (:objects
    base
	base-torso
	braiserbody#1
	braiserbody#1::braiser_bottom
	braiserlid
	counter#1
	left
	medicine#1
	right
	veggiezucchini
  )

  (:init
	;; discrete facts (e.g. types, affordances)
	(canmove)
	(canpull)

	(arm right)
	(arm left)

	(handempty right)
	(handempty left)

	(controllable left)

	(graspable braiserlid)
	(graspable veggiezucchini)
	(graspable medicine#1)

	(oftype medicine#1 @medicine)
	(oftype veggiezucchini @edible)

	(bconf q768=(2.0, 4.0, 0.2, 3.142))

	(atbconf q768=(2.0, 4.0, 0.2, 3.142))

	(surface braiserbody#1::braiser_bottom)

	(stackable braiserlid braiserbody#1::braiser_bottom)
	(stackable medicine#1 braiserbody#1::braiser_bottom)
	(stackable veggiezucchini counter#1)
	(stackable veggiezucchini braiserbody#1::braiser_bottom)
	(stackable braiserlid counter#1)
	(stackable medicine#1 counter#1)

	(pose braiserlid p179=(0.678, 2.346, 1.173, 0.0, -0.0, -2.356))
	(pose medicine#1 p178=(0.64, 2.347, 1.11, 0.0, -0.0, 3.031))
	(pose veggiezucchini p177=(0.775, 6.335, 1.079, 0.0, -0.0, 1.722))

	(atpose veggiezucchini p177=(0.775, 6.335, 1.079, 0.0, -0.0, 1.722))
	(atpose braiserlid p179=(0.678, 2.346, 1.173, 0.0, -0.0, -2.356))
	(atpose medicine#1 p178=(0.64, 2.347, 1.11, 0.0, -0.0, 3.031))

	(aconf left aq864=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(aconf right aq912=(-2.135, 1.296, -3.75, -0.15, -10000.0, -0.1, -10000.0))

	(ataconf left aq864=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(ataconf right aq912=(-2.135, 1.296, -3.75, -0.15, -10000.0, -0.1, -10000.0))

	(defaultaconf left aq864=(0.677, -0.343, 1.2, -1.467, 1.242, -1.954, 2.223))
	(defaultaconf right aq912=(-2.135, 1.296, -3.75, -0.15, -10000.0, -0.1, -10000.0))

	(supported medicine#1 p178=(0.64, 2.347, 1.11, 0.0, -0.0, 3.031) braiserbody#1::braiser_bottom)

  )

  (:goal (and
    (on braiserlid counter#1)
    (on veggiezucchini braiserbody#1::braiser_bottom)
  ))
)
        
