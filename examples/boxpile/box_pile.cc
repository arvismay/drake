#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/plant/compliant_contact_manager.h"

// #include "drake/multibody/contact_solvers/contact_solver_results.h"
// #include "drake/multibody/contact_solvers/contact_solver_utils.h"
// #include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
// #include "drake/multibody/contact_solvers/sap/sap_friction_cone_constraint.h"
// #include "drake/multibody/contact_solvers/sap/sap_solver.h"
// #include "drake/multibody/contact_solvers/sap/sap_solver_results.h"


DEFINE_double(target_realtime_rate, 1.0,
              "Rate at which to run the simulation, relative to realtime");
DEFINE_double(simulation_time, 10,
              "Desired duration of the simulation in seconds.");
// See MultibodyPlantConfig for the valid strings of contact_model.
DEFINE_string(contact_model, "hydroelastic",
              "Contact model. Options are: 'point', 'hydroelastic', "
              "'hydroelastic_with_fallback'.");
// See MultibodyPlantConfig for the valid strings of contact surface
// representation.
DEFINE_string(contact_surface_representation, "polygon",
              "Contact-surface representation for hydroelastics. "
              "Options are: 'triangle' or 'polygon'. Default is 'polygon'.");
DEFINE_double(hydroelastic_modulus, 3.0e6,
              "Hydroelastic modulus of the ball, [Pa].");
DEFINE_double(resolution_hint_factor, 0.1,
              "This scaling factor, [unitless], multiplied by the radius of "
              "the ball gives the target edge length of the mesh of the ball "
              "on the surface of its hydroelastic representation. The smaller "
              "number gives a finer mesh with more tetrahedral elements.");
DEFINE_double(dissipation, 3.0,
              "Hunt & Crossley dissipation, [s/m], for the ball");
DEFINE_double(friction_coefficient, 0.3,
              "coefficient for both static and dynamic friction, [unitless], "
              "of the ball.");
DEFINE_double(mbp_dt, 0.01,
              "The fixed time step period (in seconds) of discrete updates "
              "for the multibody plant modeled as a discrete system. "
              "Strictly positive.");

// Ball's initial spatial velocity.
DEFINE_double(vx, 0,
              "Ball's initial translational velocity in the x-axis in m/s.");
DEFINE_double(vy, 0.0,
              "Ball's initial translational velocity in the y-axis in m/s.");
DEFINE_double(vz, -7.0,
              "Ball's initial translational velocity in the z-axis in m/s.");
DEFINE_double(wx, 0.0,
              "Ball's initial angular velocity in the x-axis in degrees/s.");
DEFINE_double(wy, -10.0,
              "Ball's initial angular velocity in the y-axis in degrees/s.");
DEFINE_double(wz, 0.0,
              "Ball's initial angular velocity in the z-axis in degrees/s.");

// Ball's initial pose.
DEFINE_double(z0, 0.15, "Ball's initial position in the z-axis.");
DEFINE_double(x0, 0.0, "Ball's initial position in the x-axis.");
DEFINE_double(y0, 0.0, "Ball's initial position in the x-axis.");

namespace drake {
namespace examples {
namespace box_pile {
namespace {

using drake::multibody::SpatialVelocity;
using geometry::AddContactMaterial;
using geometry::AddCompliantHydroelasticProperties;
using geometry::ProximityProperties;
using geometry::Sphere;
using geometry::Box;
using multibody::CoulombFriction;
using multibody::MultibodyPlant;
using multibody::RigidBody;
using multibody::SpatialInertia;
using multibody::UnitInertia;
using math::RigidTransformd;
using math::RotationMatrixd;
using Eigen::Vector3d;
using drake::multibody::internal::CompliantContactManager;
// using drake::multibody::contact_solvers::internal::ContactSolverResults;
// using drake::multibody::contact_solvers::internal::MergeNormalAndTangent;
// using drake::multibody::contact_solvers::internal::SapContactProblem;
// using drake::multibody::contact_solvers::internal::SapFrictionConeConstraint;
// using drake::multibody::contact_solvers::internal::SapSolver;
// using drake::multibody::contact_solvers::internal::SapSolverParameters;
// using drake::multibody::contact_solvers::internal::SapSolverResults;
// using drake::multibody::contact_solvers::internal::SapSolverStatus;
// using drake::multibody::internal::DiscreteContactPair;


void AddSphere(std::string name, double radius, double mass, double hydroelastic_modulus,
    double dissipation, const CoulombFriction<double>& surface_friction,
    double resolution_hint_factor, MultibodyPlant<double>* plant) {

    DRAKE_DEMAND(plant != nullptr);

    // Add the ball. Let B be the ball's frame (at its center). The ball's
    // center of mass Bcm is coincident with Bo.
    const Vector3<double> p_BoBcm = Vector3<double>::Zero();
    const RigidBody<double>& ball = plant->AddRigidBody(
        name.c_str(), SpatialInertia<double>{mass, p_BoBcm, UnitInertia<double>::SolidSphere(radius)});

    // Set up mechanical properties of the ball.
    geometry::ProximityProperties ball_props;
    // ball_props.AddProperty(geometry::internal::kMaterialGroup,
    //                   "dissipation_time_constant",
    //                   0.05);
    AddContactMaterial(dissipation, {} /* point stiffness */, 
                    surface_friction, &ball_props);
    AddCompliantHydroelasticProperties(radius * resolution_hint_factor, 
                                    hydroelastic_modulus, &ball_props);
    plant->RegisterCollisionGeometry(ball, RigidTransformd::Identity(), 
                                Sphere(radius), "collision", std::move(ball_props));
    const Vector4<double> orange(1.0, 0.55, 0.0, 0.2);
    plant->RegisterVisualGeometry(ball, RigidTransformd::Identity(),
                                Sphere(radius), "visual", orange);
}

// void AddBox(std::string name, double radius, double mass, double hydroelastic_modulus,
//     double dissipation, const CoulombFriction<double>& surface_friction,
//     double resolution_hint_factor, MultibodyPlant<double>* plant) {

//     DRAKE_DEMAND(plant != nullptr);

//     // Add the box. Let B be the box's frame (at its center). The ball's
//     // center of mass Bcm is coincident with Bo.
//     const Vector3<double> p_BoBcm = Vector3<double>::Zero();
//     const RigidBody<double>& box = plant->AddRigidBody(
//         name.c_str(), SpatialInertia<double>{mass, p_BoBcm, UnitInertia<double>::SolidBox(3*radius, 3*radius, radius)});

//     // Set up mechanical properties of the box.
//     geometry::ProximityProperties box_props;
    
//     // box_props.AddProperty(geometry::internal::kMaterialGroup,
//     //                   "dissipation_time_constant",
//     //                   0.05);
//     AddContactMaterial(dissipation, {} /* point stiffness */, 
//                     surface_friction, &box_props);
//     AddCompliantHydroelasticProperties(radius * resolution_hint_factor, 
//                                     hydroelastic_modulus, &box_props);
//     plant->RegisterCollisionGeometry(box, RigidTransformd::Identity(), 
//                                 Box(3*radius, 3*radius, radius), "collision", std::move(box_props));
//     const Vector4<double> orange(1.0, 0.55, 0.0, 0.2);
//     plant->RegisterVisualGeometry(box, RigidTransformd::Identity(),
//                                 Box(3*radius, 3*radius, radius), "visual", orange);
// }

int do_main() {
    systems::DiagramBuilder<double> builder;

    multibody::MultibodyPlantConfig config;
    // We allow only discrete systems.
    DRAKE_DEMAND(FLAGS_mbp_dt > 0.0);
    config.time_step = FLAGS_mbp_dt;
    config.penetration_allowance = 1;
    config.contact_model = FLAGS_contact_model;
    config.contact_surface_representation = FLAGS_contact_surface_representation;
    auto [plant, scene_graph] = AddMultibodyPlant(config, &builder);


    // Ball's parameters.
    const double radius = 0.05;   // m
    const double mass = 0.1;      // kg
    const int gx = 4;
    const int gy = 4;
    const int gz = 6;
    for(int i = 0; i < gx * gy * gz; i++) {
        std::string name = "Ball"+std::to_string(i);
        AddSphere( name, radius, mass, FLAGS_hydroelastic_modulus, FLAGS_dissipation,
                CoulombFriction<double>{
                    // static friction (unused in discrete systems)
                    FLAGS_friction_coefficient,
                    // dynamic friction
                    FLAGS_friction_coefficient},
                FLAGS_resolution_hint_factor, &plant);
    }


     // Add the floor. Assume the frame named "Floor" is in the SDFormat file.
    drake::multibody::Parser parser(&plant);
    
    // std::string floor_file_name = FindResourceOrThrow("drake/examples/boxpile/models/floor.sdf");
    // parser.AddModelFromFile(floor_file_name);
    // plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("Floor"),
    //                 RigidTransformd::Identity());
    // // End add floor

    // Add bin
    std::string bin_file_name = FindResourceOrThrow("drake/examples/boxpile/models/bin.sdf");
    parser.AddModelFromFile(bin_file_name);
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("Bin"),
                    RigidTransformd::Identity());
    // End add bin

    // Gravity acting in the -z direction.
    plant.mutable_gravity_field().set_gravity_vector(Vector3d{0, 0, -9.81});

    plant.Finalize();


    // //---------------------------------------- Enable SAP solver
    //const ConvexSolverBase<double>* solver{nullptr};
    //CompliantContactManager<double>* manager{nullptr};
    auto owned_manager = std::make_unique<CompliantContactManager<double>>();
    CompliantContactManager<double>* manager = owned_manager.get();
    plant.SetDiscreteUpdateManager(std::move(owned_manager));
    drake::multibody::contact_solvers::internal::SapSolverParameters ssp;
    ssp.ls_max_iterations = 300;
    manager->set_sap_solver_parameters(ssp);

    // //----------------------------------------


    auto meshcat = std::make_shared<drake::geometry::Meshcat>(std::nullopt);
    drake::geometry::MeshcatVisualizerParams vis_param{};
    geometry::MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph, meshcat, vis_param);
    meshcat->AddSlider("Zval", 0, 10, 0.1, 1);

    auto diagram = builder.Build();
    auto simulator = MakeSimulatorFromGflags(*diagram);

    // Set the ball's initial pose.
    systems::Context<double>& plant_context = plant.GetMyMutableContextFromRoot(&simulator->get_mutable_context());

    int idx = 0;
    for(int i=0; i < gx; i++) {
        for(int j=0; j < gy; j++){
            for(int k=0; k < gz; k++){
                std::string name = "Ball"+std::to_string(idx);
                double x = -0.20 + 0.2*i;
                double y = -0.20 + 0.2*j;
                double z = 0.10 + 0.2*k;
                std::cout<<x<<", "<< y <<", "<< z<<std::endl;
                plant.SetFreeBodyPose(&plant_context, plant.GetBodyByName(name.c_str()), math::RigidTransformd{Vector3d(x, y, z)});
                idx++;
            }
        }
    }

    simulator->set_publish_every_time_step(true);
    simulator->set_target_realtime_rate(FLAGS_target_realtime_rate);
    simulator->Initialize();
    simulator->AdvanceTo(FLAGS_simulation_time);
    systems::PrintSimulatorStatistics(*simulator);
    return 0;
}

}  // namespace
}  // namespace ball_plate
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
    gflags::SetUsageMessage(
        "Example pile of boxes.\n");
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    return drake::examples::box_pile::do_main();
}