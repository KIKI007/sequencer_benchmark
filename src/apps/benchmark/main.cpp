//
// Created by 汪子琦 on 05.01.23.
//

#include "singlebar_benchmark.h"
#include "fourbars_benchmark.h"
#include "sixbars_benchmark.h"
#include "roboarch_benchmark.h"
#include "tenbars_benchmark.h"
#include "bridge_benchmark.h"
#include "mip_heuristics_benchmark.h"
int main()
{
    benchmark::SingleBar_BenchMark app1;
    benchmark::FourBars_BenchMark app4;
    benchmark::SixBars_BenchMark app6;
    benchmark::TenBars_BenchMark app10;


    //benchmark::Roboarch_BenchMark app;
    //    benchmark::Bridge_BenchMark app;
    benchmark::MipHeuristicApp app;

    //app.launch();
    //app10.launch();
    //app6.launch();
    //app4.launch();
    app1.launch();
    return 0;
}
