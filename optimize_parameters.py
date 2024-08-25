
import subprocess
import random
import ast
import pandas as pd
def main():
    run_cmd("rm algo_files/algo_*.so")
    init(20)
    run_algos()
    for i in range(20):
        best_params = get_best()
        run_cmd("rm algo_files/algo_*.so")
        mutate(best_params,20)
        run_algos()
    best_params = get_best()
    create_files("BestAlgo",best_params,"best_algo.cpp","best_algo.h")
    
def get_best():
    df = pd.read_csv("scores.csv")
    df = df[df["score"]==df["score"].min()]
    best_name = df["name"].values[0]
    print(f"{best_name} {df.score.values[0]}")
    s = ""
    with open(f"{best_name}_params.txt","r") as f:
        s=f.readline()
    best_params = ast.literal_eval(s)
    return best_params
def run_algos():
    cmd = "../adv-cpp-3-SmartVacuumSim/build/simulator/bin/myrobot"
    cmd += " -algo_path=algo_files -house_path=../adv-cpp-3-SmartVacuumSim/house_files -num_threads=12 -summary_only"
    run_cmd(cmd)

def init(num):
    for i in range(num):
        low_range = -1
        batt_bias_low = 0.01
        top_range = 1
        parameters = dict()
        parameters["UNVISITED_DIST"]=random.uniform(low_range,top_range)
        parameters["UNVISITED_DOCK_DIST"]=random.uniform(low_range,top_range)
        parameters["UNVISITED_BATT_BIAS"]=random.uniform(batt_bias_low,top_range)
        parameters["UNVISITED_BATT_MULT"]=random.uniform(low_range,top_range)
        parameters["DIRTY_DIST"]=random.uniform(low_range,top_range)
        parameters["DIRTY_DOCK_DIST"]=random.uniform(low_range,top_range)
        parameters["DIRTY_BATT_BIAS"]=random.uniform(batt_bias_low,top_range)
        parameters["DIRTY_BATT_MULT"]=random.uniform(low_range,top_range)
        parameters["DIRT_MULT"] = random.uniform(low_range,top_range)
        parameters["C_UNVISITED_DIST"]=random.uniform(low_range,top_range)
        parameters["C_UNVISITED_DOCK_DIST"]=random.uniform(low_range,top_range)
        parameters["C_UNVISITED_BATT_BIAS"]=random.uniform(batt_bias_low,top_range)
        parameters["C_UNVISITED_BATT_MULT"]=random.uniform(low_range,top_range)
        parameters["C_DIRTY_DIST"]=random.uniform(low_range,top_range)
        parameters["C_DIRTY_DOCK_DIST"]=random.uniform(low_range,top_range)
        parameters["C_DIRTY_BATT_BIAS"]=random.uniform(batt_bias_low,top_range)
        parameters["C_DIRTY_BATT_MULT"]=random.uniform(low_range,top_range)
        parameters["C_DIRT_MULT"] = random.uniform(low_range,top_range)
        """
        new_params = dict()
        for k,v in parameters.items():
            new_params[f"C_{k}"] = v
        parameters = {**parameters,**new_params}
        """
        algo_name = f"algo_{i}"
        with open(f"{algo_name}_params.txt","w") as f:
            f.write(str(parameters))
        create_files(algo_name,parameters)
        run_cmd(compile_string(algo_name))
    
def mutate(src_params:dict,num:int):
    for i in range(num):
        div = 0.3
        new_params = dict()
        for k,v in src_params.items():
            new_params[k] = random.normalvariate(v,div)
        
        algo_name = f"algo_{i}"
        with open(f"{algo_name}_params.txt","w") as f:
            f.write(str(new_params))
        create_files(algo_name,new_params)
        run_cmd(compile_string(algo_name))
        
def compile_string(so_name:str):
    s="g++ -std=c++20 -Wall -Wextra -Werror -pedantic -O3 -fPIC -shared"
    s+= " algo_common/PathingGraph.cpp algo_common/SmartAlgorithm.cpp algo_common/TileManager.cpp"
    s+= f" -I./ -Ialgo_common -Icommon p_algo.cpp -o algo_files/{so_name}.so"
    return s
def create_files(algo_name:str,parameters:dict,cpp_file="p_algo.cpp",header_file="p_algo.h"):
    program_h = """
#ifndef __ALGORITHM_H__
#define __ALGORITHM_H__
#include "algo_common/SmartAlgorithm.h"
class {} : public SmartAlgorithm """.format(algo_name)+"{" + """
    static constexpr double UNVISITED_DIST = {};
    static constexpr double UNVISITED_DOCK_DIST = {};
    static constexpr double UNVISITED_BATT_BIAS = {};
    static constexpr double UNVISITED_BATT_MULT = {};
    static constexpr double DIRTY_DIST = {};
    static constexpr double DIRTY_DOCK_DIST = {};
    static constexpr double DIRTY_BATT_BIAS = {};
    static constexpr double DIRTY_BATT_MULT = {};
    static constexpr double DIRT_MULT = {};
    static constexpr double C_UNVISITED_DIST = {};
    static constexpr double C_UNVISITED_DOCK_DIST = {};
    static constexpr double C_UNVISITED_BATT_BIAS = {};
    static constexpr double C_UNVISITED_BATT_MULT = {};
    static constexpr double C_DIRTY_DIST = {};
    static constexpr double C_DIRTY_DOCK_DIST = {};
    static constexpr double C_DIRTY_BATT_BIAS = {};
    static constexpr double C_DIRTY_BATT_MULT = {};
    static constexpr double C_DIRT_MULT = {};
""".format(
           parameters["UNVISITED_DIST"],parameters["UNVISITED_DOCK_DIST"],parameters["UNVISITED_BATT_BIAS"],parameters["UNVISITED_BATT_MULT"],
           parameters["DIRTY_DIST"],parameters["DIRTY_DOCK_DIST"],parameters["DIRTY_BATT_BIAS"],parameters["DIRTY_BATT_MULT"],parameters["DIRT_MULT"],
           parameters["C_UNVISITED_DIST"],parameters["C_UNVISITED_DOCK_DIST"],parameters["C_UNVISITED_BATT_BIAS"],parameters["C_UNVISITED_BATT_MULT"],
           parameters["C_DIRTY_DIST"],parameters["C_DIRTY_DOCK_DIST"],parameters["C_DIRTY_BATT_BIAS"],parameters["C_DIRTY_BATT_MULT"],parameters["C_DIRT_MULT"])+"""
protected:
    double getValidScore(const AlgoInfo& state, const AlgoTileInfo& tile_info) const override;
    std::optional<double> getCloseValidScore(const AlgoInfo& state,
                                             const AlgoTileInfo& tile_info) const override;
};
#endif  //__ALGORITHM_H__
"""
    program_cpp ="""
#include "p_algo.h" 

#include "algo_common/AlgorithmRegistration.h"
REGISTER_ALGORITHM({});

double {}::getValidScore(const AlgoInfo& state,
                                         const AlgoTileInfo& tile_info) const""".format(algo_name,algo_name)+"{"+"""
    double score = 0;
    double batt_precent = static_cast<double>(state.getBattery()) / state.getMaxBattery();
    if (tile_info.getStatus() == TileStatus::ValidUnvisited) {{
        score += tile_info.getDistance() * UNVISITED_DIST +
                 tile_info.getDockDistance() * UNVISITED_DOCK_DIST *
                     (1 / (batt_precent + UNVISITED_BATT_BIAS)) * UNVISITED_BATT_MULT;
    }} else {{
        score += (tile_info.getDistance() * DIRTY_DIST +
                  tile_info.getDockDistance() * DIRTY_DOCK_DIST *
                      (1 / (batt_precent + DIRTY_BATT_BIAS)) * DIRTY_BATT_MULT -
                  (tile_info.getDirtLevel()) * DIRT_MULT);
    }}
    return score;
}}
std::optional<double> {}::getCloseValidScore(const AlgoInfo& state,
                                                             const AlgoTileInfo& tile_info) const""".format(algo_name)+"{"+"""
    double score = 0;
    double batt_precent = static_cast<double>(state.getBattery()) / state.getMaxBattery();
    if (tile_info.getStatus() == TileStatus::ValidUnvisited) {{
        score += tile_info.getDistance() * C_UNVISITED_DIST +
                 tile_info.getDockDistance() * C_UNVISITED_DOCK_DIST *
                     (1 / (batt_precent + C_UNVISITED_BATT_BIAS)) * C_UNVISITED_BATT_MULT;
    }} else {{
        score += (tile_info.getDistance() * C_DIRTY_DIST +
                  tile_info.getDockDistance() * C_DIRTY_DOCK_DIST *
                      (1 / (batt_precent + C_DIRTY_BATT_BIAS)) * C_DIRTY_BATT_MULT -
                  (tile_info.getDirtLevel()) * C_DIRT_MULT);
    }}
    return score;
}}
""".format(algo_name,algo_name,algo_name)
    with open(cpp_file,"w") as f:
        f.write(program_cpp)
    with open(header_file,"w") as f:
        f.write(program_h)
def run_cmd(cmd,stderr=False):
    s=subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE,stderr=subprocess.STDOUT).stdout.read().decode()
    return s
if __name__=="__main__":
    main()