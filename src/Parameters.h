//Parameters    

const int yForce_MAX = 1500;    //1500
const int xForce_MAX = 2500;
const float maxSpeed = 25000;     //1800   2000
const float homeSpeed = 3000;
const float maxAccel = 20000; //20000

const long xInsert = 70000;   //From machine home
const long xStartPos = 19000;
const long xHome = -2000000;     //2000;

const long yInsert = -109000;   //From machine home
const long yStartPos = -27500;
const long yHome = 2500000;   //4000;

const int xHomeSense = 5;
const int yHomeSense = 8;

const int force_refresh = 0;

const int stepspermm = 780;
/* Sensitivity
* HIGHEST     - 64    (Too sensitive = > False positive)
* LOWEST        63    (Too insensitive = > No trigger)*/
// 7 is great for 400 mA.       
#define STALL_VALUE      0 //10 // [-64..63]