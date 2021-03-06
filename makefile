#compiler
OMPL_DIR = /usr/local
INC_CLASSES = /home/avishai/Downloads/omplapp/ompl/Workspace/ckc2d/proj_classes/
INC_PLANNERS = /home/avishai/Downloads/omplapp/ompl/Workspace/ckc2d/planners/
INC_VALIDITY = /home/avishai/Downloads/omplapp/ompl/Workspace/ckc2d/validity_checkers/
INC_RUN = /home/avishai/Downloads/omplapp/ompl/Workspace/ckc2d/run/

EIGEN_DIR = /home/avishai/Documents/eigen
KDL_DIR = /usr/local

CXX= g++
CXXFLAGS= -I${OMPL_DIR}/include -I${OMPL_DIR}/lib/x86_64-linux-gnu -I${INC_CLASSES} -I${INC_PLANNERS} -I${KDL_DIR}/include -I$(EIGEN_DIR) 
LDFLAGS=  -L${OMPL_DIR}/lib -L${OMPL_DIR}/lib/x86_64-linux-gnu -lompl -lompl_app_base -lompl_app -lboost_filesystem -lboost_system -lboost_serialization -lboost_program_options -Wl,-rpath ${OMPL_DIR}/lib/x86_64-linux-gnu -larmadillo -lorocos-kdl 
LIBS += -L/usr/lib/x86_64-linux-gnu -lboost_system

CPPVRF = ${INC_VALIDITY}verification_class.cpp

CPPGDKDL = ${INC_CLASSES}kdl_class.cpp 
CPPAPC = ${INC_CLASSES}apc_class.cpp ${INC_VALIDITY}StateValidityCheckerPCS.cpp
CPPGD = ${INC_CLASSES}kdl_class.cpp ${INC_VALIDITY}StateValidityCheckerGD.cpp
CPPHB = ${INC_CLASSES}apc_class.cpp ${INC_CLASSES}kdl_class.cpp ${INC_VALIDITY}StateValidityCheckerHB.cpp

CPP_P_PCS = ${INC_RUN}plan_PCS.cpp ${INC_PLANNERS}CBiRRT_pcs.cpp ${INC_PLANNERS}SBL_pcs.cpp ${INC_PLANNERS}RRT_pcs.cpp
CPP_P_GD = ${INC_RUN}plan_GD.cpp ${INC_PLANNERS}CBiRRT_gd.cpp ${INC_PLANNERS}SBL_gd.cpp ${INC_PLANNERS}RRT_gd.cpp
CPP_P_HB = ${INC_RUN}plan_HB.cpp ${INC_PLANNERS}CBiRRT_hb.cpp ${INC_PLANNERS}SBL_hb.cpp ${INC_PLANNERS}RRT_hb.cpp

all:

	#$(CXX) ${CPP_P_PCS} ${CPPAPC} ${CPPVRF} -o ppcs $(CXXFLAGS) $(LDFLAGS) -DPCS -std=c++11
	$(CXX) ${CPP_P_GD} ${CPPGD} ${CPPVRF} -o pgd $(CXXFLAGS) $(LDFLAGS) -DPGD -std=c++11
	#$(CXX) ${CPP_P_HB} ${CPPHB} ${CPPVRF} -o phb $(CXXFLAGS) $(LDFLAGS) -DHB -std=c++11

