//
// Created by 汪子琦 on 04.04.23.
//

#ifndef GITIGNORE_CSHAPE_EXAMPLE_H
#define GITIGNORE_CSHAPE_EXAMPLE_H

#include "algorithms/optimization_zlandmark.h"
#include "algorithms/search.h"
#include "algorithms/computeDeformation.h"

namespace examples
{
    class CShape_Example{
    public:
        std::vector<std::string> outputFileNames ={
                ROBOCRAFT_DATA_FOLDER "/examples/cshape/zlandmark.json",
        };

    public:
        void launch()
        {
            runBenchMark();
        }

        void render()
        {
            for (int solverID = 0; solverID < outputFileNames.size(); solverID++)
            {
                std::string filename = outputFileNames[solverID];
                beamAssembly = std::make_shared<frame::FrameAssembly>();
                beamAssembly->loadFromJson(filename);
                search::AssemblySequence sequence;
                sequence.loadFromFile(filename);

                std::string deform_filename = filename;
                deform_filename.replace(deform_filename.end() - 5, deform_filename.end(), "_deform.json");

                algorithms::computeDeformation(beamAssembly, sequence);
                nlohmann::json json_content;
                beamAssembly->writeToJson(json_content);
                sequence.writeToJson(json_content);
                std::ofstream fout(deform_filename);
                fout << json_content;
                fout.close();
            }
        }

        void read(){
            for (int solverID = 0; solverID < outputFileNames.size(); solverID++)
            {
                beamAssembly = std::make_shared<frame::FrameAssembly>();
                beamAssembly->loadFromJson(outputFileNames[solverID]);
                search::AssemblySequence sequence;
                sequence.loadFromFile(outputFileNames[solverID]);
                std::vector<double> complianceList;
                int numHand = 1;
                double benchmark_compliance = algorithms::runEvaluation(beamAssembly, sequence, numHand, complianceList, false);
            }
        }

        void runBenchMark()
        {
            for (int solverID = 0; solverID < outputFileNames.size(); solverID++)
            {
                beamAssembly = std::make_shared<frame::FrameAssembly>();
                beamAssembly->loadFromJson(dataFolderString + "/cshape.json");

                std::vector<int> startPartIDs = {};
                std::vector<int> endPartIDs;
                for (int jd = 0; jd < beamAssembly->beams_.size(); jd++) endPartIDs.push_back(jd);

                nlohmann::json json_output;
                search::AssemblySequence sequence, load_sequence;
                load_sequence.loadFromFile(dataFolderString + "/cshape.json");
                double time = 0;
                double compliance = 0;
                //
                if (outputFileNames[solverID].find("zlandmark") != std::string::npos)
                {
                    std::shared_ptr<dto::FrameTOSequenceSum> frameTO = std::make_shared<dto::FrameTOSequenceSum>(*beamAssembly);

                    std::vector<int> start = startPartIDs;
                    std::vector<int> end = endPartIDs;
                    std::vector<int> pre_partIDs = {};
                    std::vector<int> num_bars;
                    for(int id = 0; id < load_sequence.steps.size(); id++){
                        num_bars.push_back(load_sequence.steps[id].installPartIDs.size());
                    }

                    for(int step = 0; step + 1 < num_bars.size(); step++)
                    {
                        frameTO->setStartnEnd(start, end);
                        frameTO->sections_ = {{num_bars[step] + start.size(), num_bars[step] + start.size()}};
                        frameTO->setParameters(1E-8, 0);
                        dto::FrameTOKnitroSolver solver(frameTO);

                        solver.mip_tol = 1E-10;
                        solver.nlp_tol = 1E-12;
                        solver.maxtime = 100;
                        solver.hessian_provided = true;
                        solver.MINLP = true;
                        solver.silence = false;

                        Eigen::VectorXd x;
                        double value = solver.solve(x);

                        double sub_time = algorithms::computeShortestTimeFindingIncumbentSolution(solver);
                        time += sub_time;

                        std::vector<Eigen::VectorXd> rhos;
                        frameTO->computeRhos(x.data(), rhos);
                        search::AssemblyStep assemblyStep;
                        pre_partIDs = start;
                        start = {};
                        for(int jd = 0; jd < rhos[0].size(); jd++)
                        {
                            if(rhos[0][jd] > 0.5)
                            {
                                start.push_back(jd);
                                if(std::find(pre_partIDs.begin(), pre_partIDs.end(), jd) == pre_partIDs.end()){
                                    assemblyStep.installPartIDs.push_back(jd);
                                }
                            }
                        }
                        sequence.steps.push_back(assemblyStep);

                        Eigen::VectorXd disp;
                        beamAssembly->solveElasticity(start, {}, disp);
                        compliance += beamAssembly->computeCompliance(disp, start);
                    }

                    pre_partIDs = start;
                    search::AssemblyStep assemblyStep;
                    for(int id = 0; id < beamAssembly->beams_.size(); id++){
                        if(std::find(pre_partIDs.begin(), pre_partIDs.end(), id) == pre_partIDs.end()){
                            assemblyStep.installPartIDs.push_back(id);
                        }
                    }
                    sequence.steps.push_back(assemblyStep);
                }

                {
                    Eigen::VectorXd disp;
                    beamAssembly->solveElasticity(endPartIDs, {}, disp);
                    compliance += beamAssembly->computeCompliance(disp, endPartIDs);
                }

                // output
                beamAssembly->writeToJson(json_output);
                sequence.writeToJson(json_output);
                json_output["benchmark_compliance"] = compliance;
                json_output["benchmark_time"] = time;

                std::ofstream fout(outputFileNames[solverID]);
                fout << json_output;
                fout.close();
            }
        }

    public:
        std::string dataFolderString = ROBOCRAFT_DATA_FOLDER "/examples/model";
        std::shared_ptr<frame::FrameAssembly> beamAssembly;
    };
}
#endif //GITIGNORE_BUNNY_EXAMPLE_H
