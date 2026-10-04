#ifndef IR_CAMERA_FACADE_MODULE_H
#define IR_CAMERA_FACADE_MODULE_H


#include "../de_common/helpers/json_nlohmann.hpp"

using Json_de = nlohmann::json;

#include "../de_common/de_databus/de_facade_base.hpp"
#include "../de_common/de_databus/messages.hpp"


namespace de
{
namespace ir_camera
{
    class CIRCamera_Facade : public de::comm::CFacade_Base
    {
        public:
            //https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
            static CIRCamera_Facade& getInstance()
            {
                static CIRCamera_Facade instance;

                return instance;
            }

            CIRCamera_Facade(CIRCamera_Facade const&)         = delete;
            void operator=(CIRCamera_Facade const&)          = delete;

        
        private:

            CIRCamera_Facade()
            {
            };
        
         public:
            
            ~CIRCamera_Facade ()
            {
                
            };
                
        public:
            void sendHotColdPointsLocation(const std::string& target_party_id, const Json_de targets_location) const;
            void sendIRCameraStatus(const std::string& target_party_id, const int status) const;
            void sendCameraOverlayHotCold(const std::string& target_party_id,
                                          const float& hot_x, const float& hot_y,
                                          const float& cold_x, const float& cold_y,
                                          const float& marker_arm) const;
            void sendCameraOverlayRemove(const std::string& target_party_id) const;

        protected:

            mutable uint32_t m_overlay_send_count = 0;

    };
}
}
#endif
