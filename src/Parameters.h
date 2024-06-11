//Parameters    

const int force_MAX = 1500;
const float maxSpeed = 2500;      //1800   2000
const float maxAccel = 100000;

const long xInsert = 12500;   //16500;
const long xHome = 2000;     //2000;

const long yInsert = 13850;   //17000;
const long yHome = 3500;   //4000;

/* Sensitivity
* HIGHEST     - 64    (Too sensitive = > False positive)
* LOWEST        63    (Too insensitive = > No trigger)*/
// 7 is great for 400 mA.       
#define STALL_VALUE      7 //10 // [-64..63]    