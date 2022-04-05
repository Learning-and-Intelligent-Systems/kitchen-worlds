(define
  (domain test_kitchen_lunch_domain)

  (:predicates
    ;; discrete facts (e.g. types, affordances)
    (canmove)
    (canpull)

    (arm ?x)
    (oven ?x)
    (floor ?x)
    (basin ?x)
    (handempty ?x)
    (faucet ?x)
    (fridge ?x)
    (controllable ?x)
    (counter ?x)

    (wconf ?x)
    (inwconf ?x)

    (microwave ?x)
    (edible ?x)
    (braiserlid ?x)
    (dishwasher ?x)
    (milkbottle ?x)
    (braiserbody ?x)
    (graspable ?x)
    (knob ?x)
    (door ?x)

    (bconf ?x)
    (atbconf ?x)

    (meatturkeyleg ?x)
    (veggiecabbage ?x)
    (surface ?x)
    (cleaningsurface ?x)
    (isjointto ?x ?y)

    (position ?x ?y)
    (atposition ?x ?y)

    (heatingsurface ?x)

    (pose ?x ?y)
    (isclosedposition ?x ?y)
    (atpose ?x ?y)

    (stackable ?x ?y)
    (linkpose ?x ?y)
    (controlledby ?x ?y)
    (atlinkpose ?x ?y)

    (aconf ?x ?y)
    (ataconf ?x ?y)
    (defaultconf ?x ?y)

    (supported ?x ?xc ?y)
    (cooked ?x)
  )
)

