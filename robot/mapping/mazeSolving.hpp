#include "Graph.hpp"
#include "../robot.hpp"

namespace mtrn3100 {

String turnMotionPlan(char currHead, char newHead) {
    String headings =  "NESW";
    int diffHead = headings.indexOf(currHead) - headings.indexOf(newHead); 
    if (diffHead == 1 || diffHead == -3) {
        return "L";
    } else if (diffHead == -1 || diffHead == 3) {
        return "R";
    } else if (diffHead == -2 || diffHead == 2) {
        return "RR";
    } else {
        return "";
    }
}


}
