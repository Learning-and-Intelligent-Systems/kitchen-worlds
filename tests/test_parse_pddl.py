from config import ASSET_PATH
from os.path import join
import lisdf.components as C
from lisdf.parsing import load_all

verbose = False

def get_name(f):
    return f[f.rfind('/')+1:]

if __name__ == '__main__':

    lisdf_file = join(ASSET_PATH, 'scenes/kitchen_lunch.lisdf')
    domain_file = join(ASSET_PATH, 'pddl/test_kitchen_lunch_domain.pddl')
    problem_file = join(ASSET_PATH, 'pddl/test_kitchen_lunch.pddl')

    lisdf, domain, problem = load_all(lisdf_file, domain_file, problem_file)

    if verbose:
        for t in domain.types.values():
            print(t, end='\r')
        for v in problem.init:
            print(v.predicate.to_pddl(), end='\r')
            for arg in v.arguments:
                if isinstance(arg, C.PDDLObject):
                    print(" ", arg.name, arg.sdf_object, end='\r')
                else:
                    print(" ", arg.to_pddl(), end='\r')

    print(f'finished loading scene = {get_name(lisdf_file)}, domain = {get_name(domain_file)}, problem_file = {get_name(problem_file)}')