// Prusa Colorin (FullSpectrum): importacion de color de OBJ con deteccion automatica.
// Copia fiel de la logica de Model::read_from_file y Model::obj_import_*_deal de
// Snapmaker Orca FullSpectrum v0.9.18, adaptada a FileReader de PrusaSlicer 2.9.6.
#ifndef slic3r_ColorinObjImport_hpp_
#define slic3r_ColorinObjImport_hpp_

#include "Format/OBJ.hpp"

#include <vector>

namespace Slic3r {

class Model;
struct ObjImageMapSamplePlan;
namespace ImageMap { struct VolumeData; }

bool obj_import_vertex_color_deal(const std::vector<unsigned char>& vertex_filament_ids, const unsigned char& first_extruder_id, Model* model);
bool obj_import_face_color_deal(const std::vector<unsigned char>& face_filament_ids, const unsigned char& first_extruder_id, Model* model);
bool obj_import_image_map_deal(const std::vector<unsigned char>& filament_ids,
                               const ObjImageMapSamplePlan&      sample_plan,
                               const unsigned char&              first_extruder_id,
                               Model*                            model);
bool obj_import_persistent_image_map_deal(ImageMap::VolumeData data, const unsigned char& first_extruder_id, Model* model);

// Ejecuta el flujo de color de Orca 0.9.18 sobre un OBJ ya cargado en `model`.
// Devuelve el resultado de la importacion. `cancelled` queda en true si el usuario cancela.
bool colorin_apply_obj_color_import(Model&                       model,
                                    ObjInfo&                     obj_info,
                                    const ObjImportColorFn&      objFn,
                                    const ObjImageMapProgressFn& image_map_progress_fn,
                                    const bool&                  obj_image_map_cancelled);

} // namespace Slic3r

#endif // slic3r_ColorinObjImport_hpp_
