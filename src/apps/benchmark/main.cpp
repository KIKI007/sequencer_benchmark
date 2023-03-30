//
// Created by 汪子琦 on 05.01.23.
//

#include "singlebar_benchmark.h"
#include "fourbars_benchmark.h"
#include "sixbars_benchmark.h"
#include "roboarch_benchmark.h"
#include "tenbars_benchmark.h"
#include "bridge_benchmark.h"
int main()
{
    //benchmark::SingleBar_BenchMark app;
    benchmark::FourBars_BenchMark app4;
    benchmark::SixBars_BenchMark app6;
    //benchmark::Roboarch_BenchMark app;
    benchmark::TenBars_BenchMark app10;
    //benchmark::Bridge_BenchMark app;
    app4.launch();
    app6.launch();
    app10.launch();
    return 0;
}
