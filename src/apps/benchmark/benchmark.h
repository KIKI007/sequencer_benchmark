//
// Created by 汪子琦 on 06.01.23.
//

#ifndef PAVILLION_JSON_BENCHMARK_H
#define PAVILLION_JSON_BENCHMARK_H

#include "frame/FrameAssembly.h"
#include "search/SearchAlgorithmBeamSearch.h"
#include "search/SearchAlgorithmGreedy.h"
#include "search/StateGraphMultipleArms.h"
#include "search/PartGraphFrame.h"
#include "tbb/tick_count.h"
#include <filesystem>
#include "dto/FrameTOSequenceSum.h"
#include "dto/FrameTORangeSum.h"
#include "dto/FrameTOKnitroSolver.h"

namespace simpApp
{
class BenchMark {
public:

public:

    void launch()
    {
        getFileNames();
        runBenchMark();
        //computeResult();
    }

    void getFileNames(){
        filenames.clear();
        for (const auto & entry : std::filesystem::directory_iterator(dataFolderString))
        {
            //std::cout << entry.path().stem() << std::endl;
            if(entry.path().stem() != ".DS_Store"){
                filenames.push_back(entry.path().stem());
            }
        }
    };

    std::tuple<double, double> computeGreedy(std::shared_ptr<frame::FrameAssembly> beamAssembly,
                                             bool silence,
                                             search::AssemblySequence &sequence)
    {
        std::shared_ptr<search::PartGraphFrame> partGraph = std::make_shared<search::PartGraphFrame>(beamAssembly);
        partGraph->use_displacement_ = false;

        std::shared_ptr<search::StateGraphMultipleArms> stateGraph = std::make_shared<search::StateGraphMultipleArms>(partGraph, numHand);

        std::shared_ptr<search::SearchAlgorithmGreedy> searcher = std::make_shared<search::SearchAlgorithmGreedy>(stateGraph);
        searcher->use_max_operator_ = false;
        searcher->method = search::SearchAlgorithmGreedy::forwardMethod;

        tbb::tick_count timer = tbb::tick_count::now();
        double compliance = searcher->search(sequence);
        //std::vector<double> complianceList;
        //compliance = computeComplianceList(beamAssembly, sequence, complianceList, silence);
        double time = (tbb::tick_count::now() - timer).seconds();
        if(!silence) std::cout << "greedy" << ", " << time <<  ", " << compliance << ", " << compliance * time  << std::endl;
        return {time, compliance};
    }

    std::tuple<double, double> computeGreedyBackward(std::shared_ptr<frame::FrameAssembly> beamAssembly,
                                                     bool silence,
                                                     search::AssemblySequence &sequence)
    {
        std::shared_ptr<search::PartGraphFrame> partGraph = std::make_shared<search::PartGraphFrame>(beamAssembly);
        partGraph->use_displacement_ = false;

        std::shared_ptr<search::StateGraphMultipleArms> stateGraph = std::make_shared<search::StateGraphMultipleArms>(partGraph, numHand);

        std::shared_ptr<search::SearchAlgorithmGreedy> searcher = std::make_shared<search::SearchAlgorithmGreedy>(stateGraph);
        searcher->use_max_operator_ = false;
        searcher->method = search::SearchAlgorithmGreedy::backwardMethod;

        tbb::tick_count timer = tbb::tick_count::now();
        double compliance = searcher->search(sequence);
        //std::vector<double> complianceList;
        //computeComplianceList(beamAssembly, sequence, complianceList, silence);
        double time = (tbb::tick_count::now() - timer).seconds();
        if(!silence) std::cout << "greedy-backward" << ", " << time <<  ", " << compliance << ", " << compliance * time  << std::endl;
        return {time, compliance};
    }


    std::tuple<double, double> computeGreedyBacktracking(std::shared_ptr<frame::FrameAssembly> beamAssembly,
                                                         double maxtime,
                                                         bool silence,
                                                         search::AssemblySequence &sequence)
    {
        std::shared_ptr<search::PartGraphFrame> partGraph = std::make_shared<search::PartGraphFrame>(beamAssembly);
        partGraph->use_displacement_ = false;

        std::shared_ptr<search::StateGraphMultipleArms> stateGraph = std::make_shared<search::StateGraphMultipleArms>(partGraph, numHand);

        std::shared_ptr<search::SearchAlgorithmGreedy> searcher = std::make_shared<search::SearchAlgorithmGreedy>(stateGraph);
        searcher->use_max_operator_ = false;
        searcher->maxtime = maxtime;
        searcher->method = search::SearchAlgorithmGreedy::backtrackMethod;

        tbb::tick_count timer = tbb::tick_count::now();
        double compliance = searcher->search(sequence);
        //std::vector<double> complianceList;
        //computeComplianceList(beamAssembly, sequence, complianceList, silence);
        double time = (tbb::tick_count::now() - timer).seconds();
        if(!silence) std::cout << "greedy-backtracking" << ", " << time <<  ", " << compliance << ", " << compliance * time  << std::endl;
        return {time, compliance};
    }

    std::tuple<double, double> computeBeamSearch(std::shared_ptr<frame::FrameAssembly> beamAssembly,
                           std::vector<int> startPartIDs,
                           std::vector<int> endPartIDs,
                           int beamWidth,
                           bool silence,
                           search::AssemblySequence &sequence)
    {
        std::shared_ptr<search::PartGraphFrame> partGraph = std::make_shared<search::PartGraphFrame>(beamAssembly);
        partGraph->use_displacement_ = false;

        std::shared_ptr<search::StateGraphMultipleArms> stateGraph = std::make_shared<search::StateGraphMultipleArms>(partGraph, numHand);
        stateGraph->startPartIDs_ = startPartIDs;
        stateGraph->endPartIDs_ = endPartIDs;

        std::shared_ptr<search::SearchAlgorithmBeamSearch> searcher = std::make_shared<search::SearchAlgorithmBeamSearch>(stateGraph, beamWidth, 0.1);
        searcher->use_max_operator_ = false;

        tbb::tick_count timer = tbb::tick_count::now();
        double compliance = searcher->search(sequence);
        //std::vector<double> complianceList;
        //if(!silence) computeComplianceList(beamAssembly, sequence, complianceList, silence);
        double time = (tbb::tick_count::now() - timer).seconds();
        if(!silence) std::cout << "beam-search-" << beamWidth << ", " << time <<  ", " << compliance << ", " << compliance * time  << std::endl;
        return {time, compliance};
    }

    double computeComplianceList(std::shared_ptr<frame::FrameAssembly> beamAssembly,
                                 search::AssemblySequence &sequence,
                                 std::vector<double> &complianceList,
                                 bool silence){
        std::vector<int> subset_beam_index = {};
        complianceList.clear();
        complianceList.push_back(0);

        double result = 0;
        int prevNumBars = 0;

        for (int jd = 0; jd < sequence.steps.size(); jd++)
        {
            int numNewBars = sequence.steps[jd].installPartIDs.size();
            if(subset_beam_index.size() + numNewBars - prevNumBars > numHand)
            {
                Eigen::VectorXd displacement;
                beamAssembly->solveElasticity(subset_beam_index, {}, displacement);
                double compliance =  beamAssembly->computeCompliance(displacement, subset_beam_index);
                complianceList.push_back(compliance);
                if(!silence) std::cout << subset_beam_index.size() << ", " << compliance << std::endl;
                result += compliance;
                prevNumBars = subset_beam_index.size();
                jd  --;
            }
            else{
                subset_beam_index.insert(subset_beam_index.end(),
                                         sequence.steps[jd].installPartIDs.begin(),
                                         sequence.steps[jd].installPartIDs.end());
            }
        }

        //final
        {
            Eigen::VectorXd displacement;
            beamAssembly->solveElasticity(subset_beam_index, {}, displacement);
            double compliance =  beamAssembly->computeCompliance(displacement, subset_beam_index);
            complianceList.push_back(compliance);
            if(!silence) std::cout << subset_beam_index.size() << ", " << compliance << std::endl;
            result += compliance;
        }

        return result;
    }

    std::tuple<double, double, double> computeBnBSeq(std::shared_ptr<frame::FrameAssembly> beamAssembly,
                                                     double maxtime,
                                                     search::AssemblySequence &sequence)
    {
        std::shared_ptr<dto::FrameTOSequenceSum> frameTO = std::make_shared<dto::FrameTOSequenceSum>(*beamAssembly);
        int numPart = beamAssembly->beams_.size();
        frameTO->sections_.clear();
        for(int id = 1; id < numPart; id += numHand){
            frameTO->sections_.push_back({id, id});
        }
        frameTO->setParameters(1E-8, 0);
        dto::FrameTOKnitroSolver solver(frameTO);
        solver.mip_tol = 1E-8;
        solver.nlp_tol = 1E-10;
        solver.maxtime = maxtime;
        solver.hessian_provided = true;
        solver.MINLP = true;
        solver.silence = false;
        Eigen::VectorXd x;

        double value = solver.solve(x) * 0.5;
        double lnb = solver.data.low_bnd.back();
        double time = 0;
        for(int id = (int) solver.data.time.size() - 1; id >= 0; id --)
        {
            std::cout << solver.data.compliance[id] << " " << solver.data.low_bnd[id] << " " <<  1.05 * value << " "  << solver.data.time[id] << std::endl;
            if(solver.data.compliance[id] > 1.05 * value)
            {
                break ;
            }
            time = solver.data.time[id];
        }

        std::vector<Eigen::VectorXd> rhos;
        frameTO->computeRhos(x.data(), rhos);
        std::vector<std::vector<int>> landmarks;
        for(int id = 0; id < rhos.size(); id++)
        {
            std::vector<int> lm;
            for(int jd = 0; jd < rhos[id].size(); jd++)
            {
                if(rhos[id][jd] > 0.5) lm.push_back(jd);
            }
            landmarks.push_back(lm);
        }

        landmarks.push_back(frameTO->end_partIDs_);

        double compliance = 0;
        double final_structural_compliance = 0;
        std::vector<int> prev_partIDs = {};
        for(int id = 0; id < landmarks.size(); id++)
        {
            Eigen::VectorXd displacement;
            beamAssembly->solveElasticity(landmarks[id], {}, displacement);
            compliance += beamAssembly->computeCompliance(displacement, landmarks[id]);
            if(id + 1== landmarks.size()) final_structural_compliance = beamAssembly->computeCompliance(displacement, landmarks[id]);
            search::AssemblyStep step;
            for(int jd = 0; jd < landmarks[id].size(); jd++)
            {
                int partID = landmarks[id][jd];
                if(find(prev_partIDs.begin(), prev_partIDs.end(), partID) == prev_partIDs.end()){
                    step.installPartIDs.push_back(partID);
                }
            }
            prev_partIDs = landmarks[id];
            sequence.steps.push_back(step);
        }

        lnb += final_structural_compliance;
        std::cout << "bnb, " << time <<  ", " << compliance << ", " << value + final_structural_compliance << ", " << lnb << ", " << compliance * time  << std::endl;

        return {time, compliance, lnb};
    }

    std::tuple<double, double, double> computeBnB(std::shared_ptr<frame::FrameAssembly> beamAssembly,
                                          double maxtime,
                                          search::AssemblySequence &sequence)
    {

        std::shared_ptr<dto::FrameTORangeSum> frameTO = std::make_shared<dto::FrameTORangeSum>(*beamAssembly);
        int numPart = beamAssembly->beams_.size();
        int numStep = numPart / numHand - 1;
        if(numPart % numHand != 0) numStep += 1;

        frameTO->num_arm_ = numHand;
        frameTO->sections_ = {{0, numPart}};
        frameTO->num_steps_ = {numStep};
        frameTO->setParameters(1E-8, 0);
        dto::FrameTOKnitroSolver solver(frameTO);
        solver.mip_tol = 1E-8;
        solver.nlp_tol = 1E-10;
        solver.maxtime = maxtime;
        solver.hessian_provided = true;
        solver.MINLP = true;
        solver.silence = false;
        Eigen::VectorXd x;

        double value = solver.solve(x) * 0.5;
        double lnb = solver.data.low_bnd.back();
        double time = 0;
        for(int id = (int) solver.data.time.size() - 1; id >= 0; id --)
        {
            std::cout << solver.data.compliance[id] << " " << solver.data.low_bnd[id] << " " <<  1.05 * value << " "  << solver.data.time[id] << std::endl;
            if(solver.data.compliance[id] > 1.05 * value)
            {
                break ;
            }
            time = solver.data.time[id];
        }

        std::vector<Eigen::VectorXd> rhos;
        frameTO->computeRhos(x.data(), rhos);
        std::vector<std::vector<int>> landmarks;
        for(int id = 0; id < rhos.size(); id++)
        {
            std::vector<int> lm;
            for(int jd = 0; jd < rhos[id].size(); jd++)
            {
                if(rhos[id][jd] > 0.5) lm.push_back(jd);
            }
            landmarks.push_back(lm);
        }

        landmarks.push_back(frameTO->end_partIDs_);

        double compliance = 0;
        double final_structural_compliance = 0;
        std::vector<int> prev_partIDs = {};
        for(int id = 0; id < landmarks.size(); id++)
        {
            Eigen::VectorXd displacement;
            beamAssembly->solveElasticity(landmarks[id], {}, displacement);
            compliance += beamAssembly->computeCompliance(displacement, landmarks[id]);
            if(id + 1== landmarks.size()) final_structural_compliance = beamAssembly->computeCompliance(displacement, landmarks[id]);
            search::AssemblyStep step;
            for(int jd = 0; jd < landmarks[id].size(); jd++)
            {
                int partID = landmarks[id][jd];
                if(find(prev_partIDs.begin(), prev_partIDs.end(), partID) == prev_partIDs.end()){
                    step.installPartIDs.push_back(partID);
                }
            }
            prev_partIDs = landmarks[id];
            sequence.steps.push_back(step);
        }

        lnb += final_structural_compliance;
        std::cout << "bnb, " << time <<  ", " << compliance << ", " << value + final_structural_compliance << ", " << lnb << ", " << compliance * time  << std::endl;

        return {time, compliance, lnb};
    }

    void computeLayers(std::shared_ptr<frame::FrameAssembly> beamAssembly,
                       double maxtime,
                       int numLayers,
                       std::vector<std::vector<int>> &landmarks)
    {
        std::shared_ptr<dto::FrameTOSequenceSum> frameTO = std::make_shared<dto::FrameTOSequenceSum>(*beamAssembly);

        std::vector<int> start = {};
        std::vector<int> end = frameTO->end_partIDs_;
        int num_bar_per_step = beamAssembly->beams_.size() / numLayers;

        landmarks.push_back({});
        for(int step = 1; step < numLayers; step++)
        {
            frameTO->setStartnEnd(start, end);
            frameTO->sections_ = {
                {num_bar_per_step * ((double)step), num_bar_per_step * ((double)step )}};

            frameTO->setParameters(1E-8, 0);
            dto::FrameTOKnitroSolver solver(frameTO);

            //solver.mip_tol = complianceList[num_bar_per_step * (step)] / 10;
            //solver.nlp_tol = complianceList[num_bar_per_step * (step)] / 10;
            solver.mip_tol = 1E-10;
            solver.nlp_tol = 1E-12;
            solver.maxtime = maxtime;
            solver.hessian_provided = true;
            solver.MINLP = true;
            solver.silence = false;

            Eigen::VectorXd x;
            double value = solver.solve(x);
            std::vector<Eigen::VectorXd> rhos;
            frameTO->computeRhos(x.data(), rhos);
            for(int id = 0; id < rhos.size(); id++)
            {
                std::vector<int> lm;
                for(int jd = 0; jd < rhos[id].size(); jd++)
                {
                    if(rhos[id][jd] > 0.5) lm.push_back(jd);
                }
                landmarks.push_back(lm);
            }
            start = landmarks.back();
        }
        landmarks.push_back(end);
    }

    std::tuple<double, double> computeBottomUp(std::shared_ptr<frame::FrameAssembly> beamAssembly,
                                               double maxtime,
                                               int numLayers,
                                               search::AssemblySequence &sequence)
    {
        int bottom_up_width = 100;
        std::vector<std::vector<int>> landmarks;

        tbb::tick_count timer = tbb::tick_count::now();
        computeLayers(beamAssembly, maxtime / (numLayers - 1), numLayers, landmarks);

        for(int id = 0; id + 1 < landmarks.size(); id++)
        {
            std::vector<int> startPartIDs = landmarks[id];
            std::vector<int> endPartIDs = landmarks[id + 1];
            search::AssemblySequence tmp_sequence;
            computeBeamSearch(beamAssembly, startPartIDs, endPartIDs, bottom_up_width, true, tmp_sequence);
            sequence.steps.insert(sequence.steps.end(), tmp_sequence.steps.begin(), tmp_sequence.steps.end());
        }

        std::vector<double> compliance_list;
        double compliance = computeComplianceList(beamAssembly, sequence, compliance_list, true);
        double time = (tbb::tick_count::now() - timer).seconds();
        std::cout << "bottom-up-" << bottom_up_width << ", " << time <<  ", " << compliance << ", " << compliance * time  << std::endl;
        return {time, compliance};
    }

    void computeResult()
    {
        std::vector<std::string> folderNames = {
            //ROBOCRAFT_DATA_FOLDER "/benchmark/1-forward",
            //ROBOCRAFT_DATA_FOLDER "/benchmark/1-backward",
            //ROBOCRAFT_DATA_FOLDER "/benchmark/1-backtrack",
            //ROBOCRAFT_DATA_FOLDER "/benchmark/1-beam-100",
            //ROBOCRAFT_DATA_FOLDER "/benchmark/1-beam-1000",
            //ROBOCRAFT_DATA_FOLDER "/benchmark/1-landmark",
            //ROBOCRAFT_DATA_FOLDER "/benchmark/4-forward",
            //ROBOCRAFT_DATA_FOLDER "/benchmark/4-landmark",
            //ROBOCRAFT_DATA_FOLDER "/benchmark/4-beam-100",
            //ROBOCRAFT_DATA_FOLDER "/benchmark/4-greedy-merge",
           //ROBOCRAFT_DATA_FOLDER "/benchmark/4-landmark-new",
            //ROBOCRAFT_DATA_FOLDER "/benchmark/6-forward",
            ROBOCRAFT_DATA_FOLDER "/benchmark/10-bnb-errorbar"
            //ROBOCRAFT_DATA_FOLDER "/benchmark/6-bnb"
            //ROBOCRAFT_DATA_FOLDER "/benchmark/6-greedy-merge",
            //ROBOCRAFT_DATA_FOLDER "/benchmark/10-bnb",
            //ROBOCRAFT_DATA_FOLDER "/benchmark/10-greedy-merge",
        };


        for(auto folder: folderNames)
        {
            dataFolderString = folder;
            getFileNames();
            std::ofstream fout(dataFolderString + ".txt");
            for(int id = 0; id < filenames.size(); id++)
            {
                beamAssembly = std::make_shared<frame::FrameAssembly>();
                std::string json_file = dataFolderString + "/" + filenames[id] + ".json";
                beamAssembly->loadFromJson(json_file);
                search::AssemblySequence sequence;
                sequence.loadFromFile(json_file);
                std::vector<double> complianceList;
                double compliance = computeComplianceList(beamAssembly, sequence, complianceList, true);
                nlohmann::json json_input;
                std::ifstream fin(json_file);
                fin >> json_input;
                fout << filenames[id] << ", " << beamAssembly->beams_.size() << ", " << compliance << ", " << json_input["benchmark_time"].get<double>() << ", " << json_input["obj_lnb"].get<double>() << std::endl;
                std::cout << filenames[id] << ", " << beamAssembly->beams_.size() << ", " << compliance << ", " << json_input["benchmark_time"].get<double>() << std::endl;

            }
            fout.close();
        }
    }


    void runBenchMark()
    {

        std::vector<std::string> folderNames = {
            //ROBOCRAFT_DATA_FOLDER "/benchmark/1-forward",
            //ROBOCRAFT_DATA_FOLDER "/benchmark/1-backward",
            //ROBOCRAFT_DATA_FOLDER "/benchmark/1-backtrack",
            //ROBOCRAFT_DATA_FOLDER "/benchmark/1-beam-100",
            //ROBOCRAFT_DATA_FOLDER "/benchmark/1-beam-1000",
            ROBOCRAFT_DATA_FOLDER "/benchmark/1-bnb",
            //ROBOCRAFT_DATA_FOLDER "/benchmark/1-landmark",
            //ROBOCRAFT_DATA_FOLDER "/benchmark/4-forward",
            //ROBOCRAFT_DATA_FOLDER "/benchmark/4-landmark-new",
            //ROBOCRAFT_DATA_FOLDER "/benchmark/4-landmark-new"
//            ROBOCRAFT_DATA_FOLDER "/benchmark/6-forward",
//            ROBOCRAFT_DATA_FOLDER "/benchmark/4-bnb-new"
//            ROBOCRAFT_DATA_FOLDER "/benchmark/10-bnb-errorbar"
//            ROBOCRAFT_DATA_FOLDER "/benchmark/6-greedy-merge"
//            ROBOCRAFT_DATA_FOLDER "/benchmark/10-bnb",
            //ROBOCRAFT_DATA_FOLDER "/benchmark/10-greedy-merge",
        };


        bool startCompute = false;

        for(int id = 0; id < filenames.size(); id++)
        {
            beamAssembly = std::make_shared<frame::FrameAssembly>();
            beamAssembly->loadFromJson(dataFolderString + "/" + filenames[id] + ".json");

            if(beamAssembly->beams_.size() >= 40)
                continue;

            std::cout << filenames[id] << ", " << beamAssembly->beams_.size() << std::endl;

            std::vector<int> startPartIDs = {};
            std::vector<int> endPartIDs;
            for(int jd = 0; jd < beamAssembly->beams_.size(); jd++) endPartIDs.push_back(jd);

            std::vector<search::AssemblySequence> seqs;

            search::AssemblySequence seq0, seq1, seq2, seq3, seq4, seq5, seq6;
            std::vector<double> times;
            //auto [t0, c0] = computeGreedy(beamAssembly,  false, seq0); seqs.push_back(seq0); times.push_back(t0);
            //auto [t1, c1] = computeGreedyBackward(beamAssembly,  false, seq1);seqs.push_back(seq1);times.push_back(t1);
            //auto [t2, c2] = computeGreedyBacktracking(beamAssembly,10, false, seq2);seqs.push_back(seq2);times.push_back(t2);
//            auto [t3, c3] = computeBeamSearch(beamAssembly, startPartIDs, endPartIDs, 100, false, seq3);seqs.push_back(seq3); times.push_back(t3);
            //auto [t4, c4] = computeBeamSearch(beamAssembly, startPartIDs, endPartIDs, 1000, false, seq4);seqs.push_back(seq4); times.push_back(t4);


//            double maxtime = beamAssembly->beams_.size() > 40 ? 20 : 10;
//            maxtime = beamAssembly->beams_.size() > 60 ? 200 : maxtime;
//            maxtime = beamAssembly->beams_.size() > 100 ? 300 : maxtime;
//
//            int numLayer = beamAssembly->beams_.size() > 40 ? 4 : 3;
//            numLayer = beamAssembly->beams_.size() > 60 ? 5 : numLayer;
//            numLayer = beamAssembly->beams_.size() > 100 ? 7 : numLayer;
//            numLayer = beamAssembly->beams_.size() > 120 ? 9 : numLayer;

//            double maxtime = 20;
//            int numLayer = 3;

//            auto [t5, c5] = computeBottomUp(beamAssembly, maxtime, numLayer, seq5);seqs.push_back(seq5); times.push_back(t5);

            //double time = beamAssembly->beams_.size() >= 40 ? 800 : 300;
            double time = 10;
            auto [t6, c6, lnb] = computeBnBSeq(beamAssembly, time, seq6); seqs.push_back(seq6); times.push_back(t6);

            for(int jd = 0; jd < seqs.size(); jd++)
            {
                nlohmann::json json_output;
                beamAssembly->writeToJson(json_output);
                seqs[jd].writeToJson(json_output);
                json_output["benchmark_time"] = times[jd];
                json_output["obj_lnb"] = lnb;
                json_output["obj"] = c6;
                std::ofstream fout(folderNames[jd] + "/" + filenames[id] + ".json");
                fout << json_output;
                fout.close();
            }
        }
    }

public:
    std::string dataFolderString = ROBOCRAFT_DATA_FOLDER "/dataset";
    std::vector<std::string> filenames;
    std::shared_ptr<frame::FrameAssembly> beamAssembly;
    int numHand = 1;
};
}

#endif  //PAVILLION_JSON_BENCHMARK_H
