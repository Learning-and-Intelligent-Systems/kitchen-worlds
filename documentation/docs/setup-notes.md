# Notes

## Editors

To embed codes in this documentation, I use [https://emgithub.com/](https://emgithub.com/)

To embed Google Doc/Sheet/Slides in this documentation, go to that document, click menu bar `File > Publich to the Web > Embed > Publish`.

## Programmers

To run a test script, instead of `./test_some_script.sh`  use `./test_some_script.sh 2> /dev/null` to suppress warning from killing ghost programs.

For easier debugging and learning about unfamiliar packages (e.g. PDDLGym), I'm using PyCharm. But somehow the packages need to be imported from a different path, compared to when I'm testing my script from command line. The following code changes the import script automatically:

```python
import sys
if sys.stdin and sys.stdin.isatty():
    ## to use pddlgym from command line
    ## e.g., python run.py kitchen_extended.pddl scrambled_eggs.pddl -o obj_extended.pddl -v 0 -e 'experiments/planner_FD'
    from .pddlgym.pddlgym.parser import PDDLDomainParser, PDDLProblemParser
    from .pddlgym.pddlgym.core import PDDLEnv
else:
    ## to use pddlgym from PyCharm
    ## e.g., when testing `experiments/env.py`, or `experiments/pddlgym/pddlgym/downward_translator/normalize.py`
    from pddlgym.pddlgym.parser import PDDLDomainParser, PDDLProblemParser
    from pddlgym.pddlgym.core import PDDLEnv
```