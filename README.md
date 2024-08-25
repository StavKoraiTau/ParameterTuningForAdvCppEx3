

run from a directory in the same directory as the adv-cpp-3-SmartVacuumSim directory

or change paths here:
def run_algos():
    cmd = "../adv-cpp-3-SmartVacuumSim/build/simulator/bin/myrobot"
    cmd += " -algo_path=algo_files -house_path=../adv-cpp-3-SmartVacuumSim/house_files -num_threads=12 -summary_only"
    run_cmd(cmd)