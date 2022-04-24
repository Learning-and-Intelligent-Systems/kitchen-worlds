# Set up the Repository

To clone the LEAP repository, you need to send [Yang](mailto:ztyang@mit.edu) your GitHub username to be invited as a collaborator.

Open terminal and go to your favorite work directory (e.g., ~/Documents) to clone the project:

```
git clone git@github.com:zt-yang/leap-architecture.git
```

## Planners setup

Add FastDownward or PyperPlan as git submodules:

```
cd planners
git submodule add https://github.com/aibasel/downward.git
git submodule add https://github.com/aibasel/pyperplan.git
git submodule init
git submodule update
./downward/build.py release
```

## PDDLGym setup

To enable our LEAP agent to interact with the world through observing and replanning (as opposted to take in fully observed state and spill out a plan), we use PDDLGym for constructing a gym-like environment (with state, step, reset) from PDDL.

```
cd environments
git submodule add https://github.com/tomsilver/pddlgym
git submodule init
git submodule update
brew install swi-prolog
```

To install swi-prolog with Homebrew, I am runing terminal from Rosetta.

PDDLGym support the following subset of PDDL1.2:

* STRIPS
* Typing (hierarchical is not working for Prolog inference mode)
* Quantifiers (forall, exists)
* Disjunctions (or)
* Equality
* Constants
* Derived predicates (somehow a body of `forall` doesn't work)

It doesn't support the following extensions that our domain uses

* Conditional effects
* Action costs

So to integrate, I modified PDDLGym so that it ignores warning when handling those extensions.

Revised axiom due to `ValueError: Cannot instantiate condition: not normalized` by `pddlgym/downward_translate/pddl/conditions.py`
```prolog
( :derived (is-egg ?i ) (forall (?i - egg) (raw ?i ) ) )

## occured only once thus i replaced predicate in precondition with the axiom body
  (:derived (has-parked ?a - agent)
    (forall (?c - car) 
      (when
        (agent-owns ?a ?c)
        (parked ?c)
      )
    )
  )
```

## Study Downward Translator

to test LEAP with `run.py`, we need to comment out the last line `setup()` in `environments/pddlgym/downward_translate/options.py`