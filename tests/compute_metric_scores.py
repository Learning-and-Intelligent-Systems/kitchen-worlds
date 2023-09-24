import os

def compute_success_rate(save_dir):
    all_cases = []
    successful_cases = []
    for test_case in os.listdir(save_dir):
        if "rollout.html" in os.listdir(os.path.join(save_dir, test_case)):
            all_cases.append(test_case)
            # open the rollout.html and check if "score 100, done True" is in the html
            with open(os.path.join(save_dir, test_case, "rollout.html"), 'r') as f:
                html_content = f.read()
                if "score 100, done True" in html_content:
                    successful_cases.append(test_case)
    failed_cases = list(set(all_cases) - set(successful_cases))

    # sort key by two numbers in the string, for example semantic_spec_10_seed_0
    all_cases = sorted(all_cases, key=lambda x: (int(x.split("_")[2]), int(x.split("_")[-1])))
    successful_cases = sorted(successful_cases, key=lambda x: (int(x.split("_")[2]), int(x.split("_")[-1])))
    failed_cases = sorted(failed_cases, key=lambda x: (int(x.split("_")[2]), int(x.split("_")[-1])))

    # print(f"all_cases: {all_cases}")
    print(f"successful_cases: {successful_cases}")
    print(f"failed_cases: {failed_cases}")
    print(f"success rate: {len(successful_cases) * 1.0 / len(all_cases)}")


if __name__ == "__main__":
    save_dir = "/home/weiyu/data_drive/nsplan/0922_red_bowl/evaluation_transformer_gt_admissible"
    compute_success_rate(save_dir)