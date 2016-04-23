#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include "myrrt.hpp"

using namespace OpenRAVE;

class RRTModule : public ModuleBase
{
public:
    RRTModule(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        __description = "A simple RRT plugin.";
        RegisterCommand("rrtconnect",boost::bind(&RRTModule::RRTConnectCommand,this,_1,_2),
                        "This returns rrtconnect path");
        RegisterCommand("drawpath",boost::bind(&RRTModule::Drawpath,this,_1,_2),
                        "This returns rrtconnect path");
        RegisterCommand("execute",boost::bind(&RRTModule::ExecuteTrajectory,this,_1,_2),
                        "This returns rrtconnect path");
    }
    virtual ~RRTModule() {}
    
    bool RRTConnectCommand(std::ostream& sout, std::istream& sinput)
    {   
        EnvironmentBasePtr env;
        env = GetEnv();
        RobotBasePtr robot;
        std::vector<RobotBasePtr> pr2;
        env->GetRobots(pr2);
        robot = pr2[0];
        
        float start, goal, goalbias=0.25;
        std::vector<float> startconfig;
        std::vector<float> goalconfig;

        for(int i=0; i<7; i++){
            sinput >> start;
            startconfig.push_back(start);
        }
        
        for(int i=0; i<7; i++){
            sinput >> goal;
            goalconfig.push_back(goal);
        }

        std::vector<int> indices;
        std::vector<OpenRAVE::dReal> upperlimit,lowerlimit;
        
        indices = robot->GetActiveDOFIndices();        
        robot->GetDOFLimits(lowerlimit,upperlimit,indices);
        lowerlimit[4]= -3.14;
        upperlimit[4]= 3.14;
        lowerlimit[6]= -3.14;
        upperlimit[6]= 3.14;

        // for(int i=0; i<7; ++i){

        //     std::cout<<"indices"<<" "<<indices[i]<<std::endl;
        //     std::cout<<"lowerlimit"<<i<<" "<<lowerlimit[i]<<std::endl;
        //     std::cout<<"upperlimit"<<i<<" "<<upperlimit[i]<<std::endl;
        // }

        std::vector<float> lower(lowerlimit.begin(), lowerlimit.end());
        std::vector<float> upper(upperlimit.begin(), upperlimit.end());    
        
        NodeTree *Final_Path= new NodeTree();
        Final_Path = Final_Path->rrtgrow(startconfig,goalconfig,goalbias,upper,lower,env,robot);
        
        std::vector<float> temp,pathConfigs;
        std::vector<float>::const_iterator it;

        for( int i=Final_Path->sizeNodes()-1;i>-1;i--)
        {
            temp.assign(Final_Path->getNodes(i)->getConfig().begin(),Final_Path->getNodes(i)->getConfig().end());
            pathConfigs.push_back(temp[i]);

            std::cout<<"Nodes"<<Final_Path->sizeNodes()-i-1<<":";
            for(it=temp.begin(); it!=temp.end(); it++){
               sout<<(*it);
               sout<<" "; 
               std::cout<<(*it)<<" ";
            } 
            sout<<";";
            std::cout<<std::endl; 

        }
        // reverse(pathConfigs.begin(),pathConfigs.end());   

        
        
  
        return true;
    }

    bool Drawpath(std::ostream& sout, std::istream& sinput){
        return true;
    }

    bool ExecuteTrajectory(std::ostream& sout, std::istream& sinput){
        return true;
    }
};


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "rrtmodule" ) {
        return InterfaceBasePtr(new RRTModule(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("RRTModule");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    RAVELOG_INFO("destroying plugin\n");
}

