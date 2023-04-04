//
// Created by 汪子琦 on 03.04.23.
//

#include "bridge_example.h"
#include "fertility_example.h"
#include "roboarch_example.h"
#include "frustum_example.h"
int main(){
    examples::Bridge_Example bridgeApp;
    examples::Roboarch_Example roboarchApp;
    examples::Fertility_Example fertilityApp;
    examples::Frustum_Example frustumApp;
    fertilityApp.launch();
    fertilityApp.render();
}