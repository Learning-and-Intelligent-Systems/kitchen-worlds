import shutil

from config import ASSET_PATH, OUTPUT_PATH, PBP_PATH, TEMP_PATH
from os.path import join, isdir
import lisdf.components as C
from lisdf.parsing import load_all

verbose = False


def get_name(f):
    return f[f.rfind('/')+1:]


if __name__ == '__main__':
    """
    currently broken due to lisdf version problem, ignore this for now
        lark.exceptions.UnexpectedCharacters: No terminal matches 'f' in the current parser context, at line 197 col 5
    """

    ## the scene.lisdf file assumes that the problem directory is two levels below project directory, instead of 3
    problem_dir = join(OUTPUT_PATH, 'test_pr2_kitchen_full', '240309_123733')
    temp_dir = join(TEMP_PATH, 'test_pr2_kitchen_full_240309_123733')
    if isdir(temp_dir):
        shutil.rmtree(temp_dir)
    shutil.copytree(problem_dir, temp_dir)

    lisdf_file = join(temp_dir, 'scene.lisdf')
    problem_file = join(temp_dir, 'problem.pddl')
    domain_file = join(PBP_PATH, 'pddl_domains/mobile_v5_domain.pddl')

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
    shutil.rmtree(temp_dir)
