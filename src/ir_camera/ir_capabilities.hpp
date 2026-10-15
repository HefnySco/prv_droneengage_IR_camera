#ifndef IR_CAPABILITIES_H
#define IR_CAPABILITIES_H

/**
 * @brief The "perceive" capability advert (de.cap/1) of de_ir_camera,
 * built as a pure function so it can be validated against
 * de_common's capValidateAdvert in a unit test.
 *
 * The advert carries the per-camera perception record
 * {backend, classes[], cam_id, fps, hfov} as declared state defaults;
 * the same payload is republished as live state. It replaces the
 * legacy AI_Recognition_STATUS class-list reply.
 */

#include <string>
#include <vector>

#include "../de_common/helpers/json_nlohmann.hpp"
#include "../de_common/de_databus/cap_schema.hpp"

namespace de
{
namespace ir_camera
{

/**
 * @brief The de.cap namespace of a perceive advert: one per cam_id, so
 * several perceivers on one unit (e.g. RGB de_perceive + thermal
 * de_ir_camera) each own a namespace. "perceive_" + the cam_id
 * lower-cased with every char outside [a-z0-9_] mapped to '_' (the
 * de.cap name grammar has no '.'), capped at CAP_NAME_MAX_LEN. An empty
 * cam_id gives the bare "perceive".
 */
inline std::string perceiveNamespace (const std::string& cam_id)
{
  std::string ns = "perceive";
  if (cam_id.empty()) return ns;
  ns += '_';
  for (const char c : cam_id)
  {
    if (ns.size() >= CAP_NAME_MAX_LEN) break;
    const char l = (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c;
    const bool ok = (l >= 'a' && l <= 'z') || (l >= '0' && l <= '9') || l == '_';
    ns += ok ? l : '_';
  }
  return ns;
}

inline nlohmann::json buildPerceiveAdvert (
    const std::string& module,
    const std::string& version,
    const std::string& backend,
    const std::string& cam_id,
    const std::vector<std::string>& classes,
    const double fps,
    const double hfov)
{
  using Json_de = nlohmann::json;

  const Json_de advert = Json_de::object(
      {{"schema", "de.cap/1"},
       {"ns", perceiveNamespace(cam_id)},
       {"module", module},
       {"ver", version},
       {"state",
        Json_de::object(
            {{"cam_id",
              Json_de::object(
                  {{"type", "string"},
                   {"default", cam_id},
                   {"desc", "cam_id the published observations come from"}})},
             {"backend",
              Json_de::object(
                  {{"type", "string"},
                   {"default", backend},
                   {"desc", "perception backend (thermal, ...)"}})},
             {"classes",
              Json_de::object(
                  {{"type", "array"},
                   {"items", Json_de::object({{"type", "string"}})},
                   {"default", classes},
                   {"desc", "class labels this perceiver can report (obs cls)"}})},
             {"fps",
              Json_de::object(
                  {{"type", "number"},
                   {"min", 0.0},
                   {"default", fps},
                   {"desc", "processed frames per second; 0 = unknown"}})},
             {"hfov",
              Json_de::object(
                  {{"type", "number"},
                   {"min", 0.0},
                   {"default", hfov},
                   {"desc", "horizontal field of view in degrees; 0 = unknown"}})}})}});

  return advert;
}

/**
 * @brief The matching live-state payload for publishState(perceiveNamespace(cam_id), ...).
 */
inline nlohmann::json perceiveStatePayload (
    const std::string& cam_id,
    const std::string& backend,
    const std::vector<std::string>& classes,
    const double fps,
    const double hfov)
{
  return nlohmann::json::object(
      {{"cam_id", cam_id},
       {"backend", backend},
       {"classes", classes},
       {"fps", fps},
       {"hfov", hfov}});
}

}
}

#endif
