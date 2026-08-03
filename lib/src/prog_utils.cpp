#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
// #include <string>
// #include <stdexcept>

#include "yaml-cpp/yaml.h"

#include "last_letter_lib/prog_utils.hpp"

namespace last_letter_lib
{
    namespace programming_utils
    {

        string vectorToString2(Eigen::Vector3d vec3d, const string delimiter)
        {
            vector<double> vec(3);
            vec[0] = vec3d[0];
            vec[1] = vec3d[1];
            vec[2] = vec3d[2];
            return vectorToString2(vec, delimiter);
        }

        string vectorToString2(Eigen::Quaterniond quat, const string delimiter)
        {
            vector<double> vec(4);
            vec[0] = quat.w();
            vec[1] = quat.x();
            vec[2] = quat.y();
            vec[2] = quat.z();
            return vectorToString2(vec, delimiter);
        }

        // Find if string contains a character.
        // Test method taken from https://stackoverflow.com/questions/23433095/how-to-check-if-a-char-is-in-a-string-in-c
        bool contains(const std::string &str, char del)
        {
            return str.find(del) != std::string::npos;
        }

        bool startsWith(std::string mainStr, std::string startStr)
        {
            if (mainStr.find(startStr) == 0)
                return true;
            else
                return false;
        }

        string removePrefix(std::string mainStr, std::string startStr)
        {
            if (startsWith(mainStr, startStr))
            {
                string newString = mainStr.erase(0, startStr.size());
                return newString;
            }
            else
                return mainStr;
        }

        ///////////////////
        // ParameterManager

        // The YAML tree, hidden from prog_utils.hpp so that yaml-cpp stays out
        // of the installed headers.
        struct ParameterManager::Impl
        {
            YAML::Node node;
        };

        namespace
        {
            // Returns a null YAML::Node if it doesn't exist.
            YAML::Node find_parameter(const YAML::Node node, const string param_name_p)
            {
                // Split the name into '/'
                std::vector<std::string> names;
                split_string(param_name_p, names, '/');

                // Recursively iterate into the tree structure.
                YAML::Node temp_node;
                temp_node.reset(node);
                for (auto name : names)
                {
                    if (temp_node[name])
                    {
                        temp_node.reset(temp_node[name]);
                    }
                    else
                    {
                        YAML::Node temp_node;
                        return temp_node["invalid"];
                    }
                }
                return temp_node;
            }

            vector<string> get_keys(YAML::Node n)
            {
                vector<string> v;
                for (YAML::const_iterator it = n.begin(); it != n.end(); ++it)
                {
                    if (it->second.IsMap())
                    {
                        vector<string> child_keys = get_keys(it->second.as<YAML::Node>());
                        for (auto k : child_keys)
                        {
                            v.push_back(it->first.as<string>() + '/' + k);
                        }
                    }
                    else
                    {
                        v.push_back(it->first.as<string>());
                    }
                }
                return v;
            }

            // Only loads a parameter if it already exists in own.
            void load_parameters_into(YAML::Node own, YAML::Node n)
            {
                vector<string> parameter_list = get_keys(own);
                for (auto param_name : parameter_list)
                {
                    YAML::Node param_node = find_parameter(n, param_name);
                    if (param_node)
                    {
                        YAML::Node own_param_node = find_parameter(own, param_name);
                        own_param_node = YAML::Node(param_node);
                    }
                }
            }
        } // namespace

        ParameterManager::ParameterManager(string name_p)
            : name{name_p}, impl_{std::make_shared<Impl>()}
        {
            impl_->node["name"] = name_p;
        }

        ParameterManager::ParameterManager(string name_p, std::shared_ptr<Impl> impl)
            : name{name_p}, impl_{impl}
        {
        }

        // These factories allow loading non-existent parameters: the document
        // replaces the (empty) tree wholesale instead of being filtered against
        // it the way load_file / load_stream are.
        ParameterManager ParameterManager::from_yaml_string(string name_p, const string &yaml_str)
        {
            auto impl = std::make_shared<Impl>();
            impl->node = YAML::Load(yaml_str);
            return ParameterManager(name_p, impl);
        }

        ParameterManager ParameterManager::from_yaml_file(string name_p, const string &filepath)
        {
            auto impl = std::make_shared<Impl>();
            impl->node = YAML::LoadFile(filepath);
            return ParameterManager(name_p, impl);
        }

        void ParameterManager::load_file(string filepath)
        {
            auto n = YAML::LoadFile(filepath);
            load_parameters_into(impl_->node, n);
        }

        void ParameterManager::load_stream(const std::string yaml_str)
        {
            auto n = YAML::Load(yaml_str);
            load_parameters_into(impl_->node, n);
        }

        void ParameterManager::load_parameters(ParameterManager params_p)
        {
            load_parameters_into(impl_->node, params_p.impl_->node);
        }

        void ParameterManager::register_child_mngr(ParameterManager child_mngr)
        {
            impl_->node[child_mngr.name] = child_mngr.impl_->node;
        }

        bool ParameterManager::exists(const string param_name_p)
        {
            return (bool)find_parameter(impl_->node, param_name_p);
        }

        ParameterManager ParameterManager::filter(const string prefix)
        {
            // // Append a trailing / for completeness.
            // if ((char)prefix.back() != '/')
            // {
            //     prefix += '/';
            // }
            if (exists(prefix))
            {
                // Split the name into '/'
                std::vector<std::string> names;
                split_string(prefix, names, '/');
                string name{names.back()};
                auto impl = std::make_shared<Impl>();
                impl->node = find_parameter(impl_->node, prefix);
                return ParameterManager(name, impl);
            }
            else
            {
                throw std::invalid_argument(string("Parameter prefix ") + prefix + string(" does not exist."));
            }
        }

        vector<string> ParameterManager::keys()
        {
            return get_keys(impl_->node);
        }

        string ParameterManager::str()
        {
            std::stringstream buffer;
            buffer << impl_->node;
            return buffer.str();
        }

        template <typename T>
        T ParameterManager::get(const string &param_name)
        {
            if (exists(param_name))
            {
                YAML::Node temp_node{find_parameter(impl_->node, param_name)};
                return temp_node.as<T>();
            }
            else
            {
                throw std::invalid_argument(string("Parameter ") + param_name + string(" does not exist."));
            }
        }

        template <typename T>
        bool ParameterManager::set(const string param_name, const T value, bool safe)
        {
            // std::cout << "Setting parameter " << param_name << std::endl;
            bool param_exists = exists(param_name);
            if (param_exists || !safe)
            {
                std::vector<std::string> names;
                split_string(param_name, names, '/');
                YAML::Node current_node;
                current_node.reset(impl_->node);
                for (auto name : names)
                {
                    if (current_node[name])
                    {
                    }
                    else
                    {
                        current_node[name] = YAML::Node();
                    }
                    current_node.reset(current_node[name]);
                }
                current_node = value;
                return true;
            }
            else
            {
                return false;
            }
        }

        // Explicit instantiations, to keep yaml-cpp a hidden implementation.
        template bool ParameterManager::get<bool>(const string &);
        template int ParameterManager::get<int>(const string &);
        template uint ParameterManager::get<uint>(const string &);
        template double ParameterManager::get<double>(const string &);
        template std::string ParameterManager::get<std::string>(const string &);
        template std::vector<int> ParameterManager::get<std::vector<int>>(const string &);
        template std::vector<double> ParameterManager::get<std::vector<double>>(const string &);

        template bool ParameterManager::set<bool>(const string, const bool, bool);
        template bool ParameterManager::set<int>(const string, const int, bool);
        template bool ParameterManager::set<uint>(const string, const uint, bool);
        template bool ParameterManager::set<double>(const string, const double, bool);
        template bool ParameterManager::set<std::string>(const string, const std::string, bool);
        template bool ParameterManager::set<std::vector<int>>(const string, const std::vector<int>, bool);
        template bool ParameterManager::set<std::vector<double>>(const string, const std::vector<double>, bool);

        // Filter a YAML::Node file to keep only a sub-parameter set
        // Example: for start string /world, keep /world/timeControls but not /environment/rho
        // YAML::Node filterConfig(YAML::Node config, std::string prefix)
        // {
        //     YAML::Node filteredNode;
        //     for (YAML::const_iterator it = config.begin(); it != config.end(); ++it)
        //     {
        //         string paramName = it->first.as<string>();
        //         if (startsWith(paramName, prefix))
        //         {
        //             string newName = paramName.erase(0, prefix.size());
        //             filteredNode[newName] = it->second;
        //         }
        //         else
        //         {
        //         }
        //     }
        //     return filteredNode;
        // }

        // Randomize the requested parameters of a configuration node by std_dev
        // YAML::Node randomizeConfig(YAML::Node config, vector<string> stringVec, double std_dev)
        // {
        //     if (std_dev < 0)
        //     {
        //         throw domain_error("Standard deviation cannot be negative");
        //     }

        //     std::random_device rdev;
        //     std::default_random_engine generator;
        //     generator.seed(rdev());
        //     std::normal_distribution<double> distribution(0.0, std_dev);
        //     YAML::Node newConfig(config);

        //     if (std_dev) // Apply only if std_dev>0
        //     {
        //         double scalarParam;
        //         for (auto const &paramString : stringVec)
        //         {
        //             if (!(config[paramString].Type() == YAML::NodeType::Scalar))
        //             {
        //                 throw runtime_error("Requested parameter " + paramString + " is not scalar.");
        //             }
        //             cout << "Randomizing parameter: " << paramString << endl;
        //             getParameter(config, paramString, scalarParam);
        //             newConfig[paramString] = scalarParam * (1 + distribution(generator));
        //         }
        //     }
        //     return newConfig;
        // }

        // ConfigsStruct_t randomizeConfigsStruct(const ConfigsStruct_t p_configStruct, const YAML::Node randomizerConfig)
        // {
        //     ConfigsStruct_t randomizedConfig;
        //     double std_dev;
        //     getParameter(randomizerConfig, "std_dev", std_dev);
        //     if (!std_dev) // No changes required
        //     {
        //         return p_configStruct;
        //     }
        //     else
        //     {
        //         vector<string> paramList;

        //         getParameter(randomizerConfig, "aerodynamics", paramList);
        //         randomizedConfig.aero = randomizeConfig(p_configStruct.aero, paramList, std_dev);

        //         paramList.clear();
        //         getParameter(randomizerConfig, "ground", paramList);
        //         randomizedConfig.ground = randomizeConfig(p_configStruct.ground, paramList, std_dev);

        //         paramList.clear();
        //         getParameter(randomizerConfig, "propulsion", paramList);
        //         randomizedConfig.prop = randomizeConfig(p_configStruct.prop, paramList, std_dev);

        //         paramList.clear();
        //         getParameter(randomizerConfig, "inertial", paramList);
        //         randomizedConfig.inertial = randomizeConfig(p_configStruct.inertial, paramList, std_dev);

        //         paramList.clear();
        //         getParameter(randomizerConfig, "init", paramList);
        //         randomizedConfig.init = randomizeConfig(p_configStruct.init, paramList, std_dev);

        //         return randomizedConfig;
        //     }
        // }

        ParameterManager loadModelConfig(string modelName, string modelsRoot)
        {
            string modelFolderName = "last_letter_models/";
            string aircraftDir = "aircraft/";
            // An empty root assumes the default, $HOME/last_letter_models.
            string modelPath = modelsRoot.empty()
                                   ? getHomeFolder() + "/" + modelFolderName
                                   : modelsRoot;
            if (modelPath.back() != '/')
            {
                modelPath += '/';
            }

            ParameterManager configs("aircraft");

            string prop_filename = "propulsion.yaml";
            string aero_filename = "aerodynamics.yaml";
            string ground_filename = "ground.yaml";
            string inertial_filename = "inertial.yaml";
            string init_filename = "init.yaml";
            string world_filename = "world.yaml";
            string environment_filename = "environment.yaml";
            string randomizer_filename = "randomizer.yaml";

            string fullWorldFilename = modelPath + world_filename;
            string fullEnvironmentFilename = modelPath + environment_filename;
            string fullPropFilename = modelPath + aircraftDir + modelName + "/" + prop_filename;
            string fullAeroFilename = modelPath + aircraftDir + modelName + "/" + aero_filename;
            string fullGroundFilename = modelPath + aircraftDir + modelName + "/" + ground_filename;
            string fullInitFilename = modelPath + aircraftDir + modelName + "/" + init_filename;
            string fullRandomizerFilename = modelPath + aircraftDir + modelName + "/" + randomizer_filename;

            ParameterManager world = ParameterManager::from_yaml_file("world", fullWorldFilename);
            configs.register_child_mngr(world);
            ParameterManager env = ParameterManager::from_yaml_file("env", fullEnvironmentFilename);
            configs.register_child_mngr(env);

            ParameterManager dynamics_config("dynamics");
            ParameterManager prop = ParameterManager::from_yaml_file("prop", fullPropFilename);
            dynamics_config.register_child_mngr(prop);
            ParameterManager aero = ParameterManager::from_yaml_file("aero", fullAeroFilename);
            dynamics_config.register_child_mngr(aero);
            ParameterManager ground = ParameterManager::from_yaml_file("ground", fullGroundFilename);
            dynamics_config.register_child_mngr(ground);
            configs.register_child_mngr(dynamics_config);

            ParameterManager kinematics_config("kinematics");
            configs.register_child_mngr(kinematics_config);

            ParameterManager init = ParameterManager::from_yaml_file("init", fullInitFilename);
            configs.register_child_mngr(init);

            return configs;
            // Load randomization information, and apply them
            // YAML::Node randomizerConfig = YAML::LoadFile(fullRandomizerFilename);

            // return randomizeConfigsStruct(configs, randomizerConfig);
        }

        // Return home folder, without trailling slash.
        string getHomeFolder()
        {
            passwd *pw = getpwuid(getuid());
            string path(pw->pw_dir);
            return path;
        }


        ///////////////
        // Parametrized

        Parametrized::Parametrized(string name_p) : params_(name_p)
        {
            set_param("name", name_p, false);
            name = name_p;
        };

        void Parametrized::initialize(ParameterManager params_p)
        {
            initialize_parameters();
            load_parameters(params_p);
            update_parameters();
        }

        void Parametrized::initialize(const std::string yaml_str)
        {
            initialize_parameters();
            params_.load_stream(yaml_str);
            update_parameters();
        }

        /////////////////////////////////////////////////////////////////
        // Build a new polynomial, reading from a configuration YAML Node
        math_utils::Polynomial *buildPolynomial(ParameterManager param_mng)
        {
            int polyType;
            polyType = param_mng.get<int>("polyType");
            switch (polyType)
            {
            case 0:
            {
                // std::cout << "selecting 1D polynomial" << std::endl;
                uint polyNo;
                polyNo = param_mng.get<uint>("polyNo");
                std::vector<double> coeffVect;
                coeffVect = param_mng.get<std::vector<double>>("coeffs");
                if (coeffVect.size() != (polyNo + 1))
                {
                    throw runtime_error("Parameter array coeffs size did not match polyNo");
                }
                double coeffs[polyNo + 1];
                std::copy(coeffVect.begin(), coeffVect.end(), coeffs);
                return new math_utils::Polynomial1D(polyNo, coeffs);
            }
            case 1:
            {
                // std::cout << "selecting 2D polynomial" << std::endl;
                std::vector<int> polyNoVect;
                polyNoVect = param_mng.get<std::vector<int>>("polyNo");
                std::vector<double> coeffVect;
                uint polyOrder1 = coeffVect[0];
                uint polyOrder2 = coeffVect[1];
                coeffVect = param_mng.get<std::vector<double>>("coeffs");
                if (coeffVect.size() != ((2 * polyOrder2 + 2 * polyOrder1 * polyOrder2 + polyOrder1 - polyOrder1 * polyOrder1 + 2) / 2))
                {
                    throw runtime_error("Parameter array coeffs size did not match polyNo");
                }
                double coeffs[coeffVect.size()];
                std::copy(coeffVect.begin(), coeffVect.end(), coeffs);
                return new math_utils::Polynomial2D(polyOrder1, polyOrder2, coeffs);
            }
            case 2:
            {
                // std::cout << "selecting cubic spline" << std::endl;
                uint breaksNo;
                breaksNo = param_mng.get<uint>("breaksNo");
                std::vector<double> breaksVect;
                breaksVect = param_mng.get<std::vector<double>>("breaks");
                if (breaksVect.size() != (breaksNo + 1))
                {
                    throw runtime_error("Spline breaks order and provided breaks number do not match");
                }
                double breaks[breaksVect.size()];
                std::copy(breaksVect.begin(), breaksVect.end(), breaks);
                std::vector<double> coeffVect;
                coeffVect = param_mng.get<std::vector<double>>("coeffs");
                if (breaksNo * 4 != coeffVect.size())
                {
                    throw runtime_error("breaks order and provided coeffs number do not match");
                }
                double coeffs[coeffVect.size()];
                std::copy(coeffVect.begin(), coeffVect.end(), coeffs);
                return new math_utils::Spline3(breaksNo, breaks, coeffs);
            }
            default:
            {
                throw runtime_error("Error while constructing a polynomial");
                break;
            }
            }
        }

        TogglerInput::TogglerInput(const uint8_t num_states)
        {
            num_states_ = num_states;
        }

        void TogglerInput::set_state(const uint8_t state)
        {
            if (!(state > (num_states_ - 1)))
            {
                state_ = state;
            }
        }

        void TogglerInput::set_threshold(const double threshold)
        {
            threshold_ = threshold;
        }

        void TogglerInput::read_input(const double input)
        {
            if (input > threshold_)
            {
                if (input_active_)
                {
                    // Do nothing
                }
                else
                {
                    state_++;
                    input_active_ = true;
                }
            }
            else // Input is not pressed
            {
                input_active_ = false;
            }

            if (state_ > (num_states_ - 1))
            {
                state_ = 0;
            }
        }

        uint8_t TogglerInput::get_state()
        {
            return state_;
        }

        double TogglerInput::get_state_normalized() // Return state as 0-1 value
        {
            if (num_states_ == 1)
            {
                return 0;
            }
            else
            {
                return (double)(state_) / (num_states_ - 1);
            }
        }

        TogglerPwm::TogglerPwm(const uint8_t num_states)
            : toggler_(num_states)
        {
        }

        void TogglerPwm::set_state(const uint8_t state)
        {
            toggler_.set_state(state);
        }

        void TogglerPwm::set_threshold(const double threshold)
        {
            threshold_pwm_ = threshold;
        }
        void TogglerPwm::read_input(const unsigned int input)
        {
            if (input > threshold_pwm_)
            {
                toggler_.read_input(1);
            }
        }
        uint8_t TogglerPwm::get_state()
        {
            return toggler_.get_state();
        }
        double TogglerPwm::get_state_pwm() // Return state as 1000-2000 value
        {
            return 1000 + toggler_.get_state_normalized() * 1000;
        }

    } // namespace programming_utils
} // namespace last_letter_lib
