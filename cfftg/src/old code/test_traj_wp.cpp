#include "utility.h"







int main (int argc, char** argv) {
 ros::init (argc, argv, "test_traj");
 ros::NodeHandle nh;
 
 std::vector<float> y{-0.0207399, -0.0207733, -0.0208725, -0.021036, -0.0212622, -0.0215492, -0.0218949, -0.0222974, -0.0227542, -0.0232628, -0.0238205, -0.0244245, -0.0250715, -0.0257582, -0.0264811, -0.0272363, -0.0280198, -0.0288274, -0.0296543, -0.0304959, -0.0313469, -0.0322022, -0.0330559, -0.0339023, -0.0347353, -0.0355484, -0.0363351, -0.0370886, -0.037802, -0.0384681, -0.0390798, -0.0396298, -0.0401108, -0.0405156, -0.040837, -0.041068, -0.0412018, -0.0412318, -0.0411518, -0.0409559, -0.0406387, -0.0401951, -0.0396207, -0.0389115, -0.038064, -0.0370756, -0.0359438, -0.0346669, -0.0332438, -0.0316739, -0.0299569, -0.0280931, -0.0260834, -0.0239289, -0.0216311, -0.0191918, -0.0166131, -0.0138973, -0.0110471, -0.008065, -0.00495399, -0.00171699, 0.00164294, 0.00512271, 0.00871917, 0.0124291, 0.0162495, 0.0201769, 0.0242084, 0.0283408, 0.0325709, 0.0368957, 0.0413122, 0.0458174, 0.0504085, 0.0550824, 0.0598365, 0.064668, 0.0695741, 0.0745522, 0.0795998, 0.0847143, 0.0898933, 0.0951343, 0.100435, 0.105793, 0.111206, 0.116673, 0.12219, 0.127756, 0.133368, 0.139026, 0.144726, 0.150468, 0.156249, 0.162067, 0.167922, 0.173811, 0.179732, 0.185685, 0.191668, 0.19768, 0.203718, 0.209783, 0.215873, 0.221986, 0.228122, 0.23428, 0.240459, 0.246658, 0.252876, 0.259113, 0.265367, 0.271639, 0.277928, 0.284234, 0.290556, 0.296894, 0.303248, 0.309618, 0.316004, 0.322405, 0.328823, 0.335257, 0.341708, 0.348175, 0.354661, 0.361164, 0.367686, 0.374228, 0.380791, 0.387374, 0.39398, 0.400609, 0.407241, 0.413801, 0.420285, 0.426692, 0.433022, 0.439274, 0.44545, 0.451548, 0.457568, 0.46351, 0.469374, 0.47516, 0.480868, 0.486497, 0.492047, 0.497518, 0.50291, 0.508223, 0.513456, 0.51861, 0.523684, 0.528678, 0.533592, 0.538427, 0.543181, 0.547854, 0.552448, 0.556961, 0.561394, 0.565746, 0.570018, 0.57421, 0.578321, 0.582351, 0.586301, 0.590171, 0.593961, 0.59767, 0.601299, 0.604848, 0.608317, 0.611707, 0.615017, 0.618247, 0.621399, 0.624471, 0.627464, 0.630379, 0.633216, 0.635974, 0.638655, 0.641259, 0.643785, 0.646235, 0.648608, 0.650905, 0.653126, 0.655273, 0.657344, 0.659341, 0.661264, 0.663114, 0.664891, 0.666596, 0.668229, 0.66979, 0.671281, 0.672701, 0.674052, 0.675334, 0.676547, 0.677693, 0.678772, 0.679784, 0.680731, 0.681612, 0.682429, 0.683183, 0.683874, 0.684502, 0.685069, 0.685575, 0.686022, 0.686409, 0.686738, 0.687009, 0.687223, 0.687381, 0.687484, 0.687532, 0.687526, 0.687467, 0.687356, 0.687194, 0.686981, 0.686717, 0.686404, 0.686043, 0.685634, 0.685178, 0.684675, 0.684126, 0.683532, 0.682893, 0.68221, 0.681484, 0.680715, 0.679904, 0.67905, 0.678155, 0.677219, 0.676242, 0.675225, 0.674168, 0.673072, 0.671935, 0.67076, 0.669545, 0.668292, 0.666999, 0.665668, 0.664297, 0.662888, 0.66144, 0.659989, 0.658723, 0.657667, 0.656816, 0.656167, 0.655715, 0.655455, 0.655382, 0.655491, 0.655774, 0.656226, 0.65684, 0.657609, 0.658527, 0.659584, 0.660775, 0.662089, 0.663521, 0.66506, 0.666698, 0.668426, 0.670235, 0.672116, 0.674059, 0.676054, 0.678091, 0.68016, 0.68225, 0.684352, 0.686454, 0.688546, 0.690617, 0.692655, 0.69465, 0.69659, 0.698465, 0.700263, 0.701973, 0.703584, 0.705086, 0.706467, 0.707717, 0.708826, 0.709784, 0.710581, 0.711208, 0.711655, 0.711915, 0.711979, 0.71184, 0.711491, 0.710924, 0.710136, 0.709119, 0.707869, 0.706382, 0.704656, 0.702686, 0.700471, 0.698009, 0.6953, 0.692343, 0.689139, 0.685688, 0.681993, 0.678055, 0.673877, 0.669463, 0.664816, 0.659941, 0.654843, 0.649527, 0.643999, 0.638265, 0.632332, 0.626207, 0.619897, 0.613409, 0.606753, 0.599936, 0.592967, 0.585855, 0.578609, 0.571238, 0.563752, 0.556161, 0.548474, 0.540703, 0.532857, 0.524947, 0.516983, 0.508977, 0.500938, 0.49288, 0.484812, 0.476746, 0.468694, 0.460667, 0.452678, 0.444737, 0.436857, 0.429051, 0.421331, 0.413708, 0.406196, 0.398807, 0.391555, 0.384452, 0.37751, 0.370744, 0.364166, 0.357788, 0.351625, 0.345687, 0.339989, 0.33454, 0.329354, 0.324439, 0.319806, 0.315462, 0.311415, 0.307668, 0.304226, 0.301088, 0.298252, 0.295715, 0.293467, 0.2915, 0.289799, 0.288349, 0.287132, 0.286128, 0.285314, 0.28467, 0.284171, 0.283795, 0.28352, 0.283326, 0.283195, 0.283111, 0.28306, 0.283033, 0.283025};

 float temp=0.0;
 std::vector<float> x;
 std::vector<Point> traj;
 for(int i=0;i< y.size();i++){
	x.push_back(temp);
	traj.push_back(Point(x[i],y[i]));
	temp+=0.01;
 }
 std::vector<Point> wps = select_waypoints(traj);
 std::vector<float> wps_y;
 std::vector<float> wps_x;
 for(int i=0;i<wps.size();i++){
	std::cout<<wps[i].getx()<<std::endl;
	wps_y.push_back(wps[i].gety());
	wps_x.push_back(wps[i].getx());
 }
 std::vector<float> velocities = derivative(wps_y);
 std::vector<float> timing = derivative(wps_x);
 for(int i=0;i<velocities.size();i++){
	std::cout<<velocities[i] / timing[i] <<std::endl;
 }
 
}