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
    //benchmark::FourBars_BenchMark app;
    //benchmark::SixBars_BenchMark app;
    //benchmark::Roboarch_BenchMark app;
    //benchmark::TenBars_BenchMark app;
    benchmark::Bridge_BenchMark app;
    app.launch();
    return 0;
}
