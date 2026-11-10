// SPDX-FileCopyrightText: Copyright (c) Kitware Inc.
// SPDX-License-Identifier: BSD-3-Clause
/**
 * @class   vtkPVXRInterfaceWidgets
 * @brief   Support for widgets in XR (text billboard, measuring tool, etc.).
 *
 */

#ifndef vtkPVXRInterfaceWidgets_h
#define vtkPVXRInterfaceWidgets_h

#include "vtkNew.h" // for ivars
#include "vtkObject.h"
#include "vtkSmartPointer.h" // for ivars

#include <functional> // for std::function
#include <future>     // for ivar
#include <vector> // for ivar

class vtkBoxWidget2;
class vtkDataSetAttributes;
class vtkEventData;
class vtkEventDataDevice3D;
class vtkImageData;
class vtkImplicitPlaneWidget2;
class vtkOpenGLRenderWindow;
class vtkPVXRInterfaceHelper;
class vtkProp;
struct vtkPVXRInterfaceHelperLocation;
class vtkStringArray;
class vtkTransform;

class vtkPVXRInterfaceWidgets : public vtkObject
{
public:
  static vtkPVXRInterfaceWidgets* New();
  vtkTypeMacro(vtkPVXRInterfaceWidgets, vtkObject);

  /**
   * Set whether to display a navigation panel in the given render window.
   */
  void SetShowNavigationPanel(bool val, vtkOpenGLRenderWindow* renderWindow);

  /**
   * Get whether the navigation panel is enabled or not.
   */
  bool GetNavigationPanelVisibility();

  /**
   * Update information in the navigation panel based on the position of the given device.
   */
  void UpdateNavigationText(vtkEventDataDevice3D* edd, vtkOpenGLRenderWindow* renderWindow);

  /**
   * Enable measurement tool in the given render window.
   */
  void TakeMeasurement(vtkOpenGLRenderWindow* renWin);

  /**
   * Disable and remove measurement tool.
   */
  void RemoveMeasurement();

  /**
   * Return whether the distance widget is enabled or not.
   */
  bool IsMeasurementEnabled() const;

  /**
   * Show a billboard with the provided text and texture file.
   * Its position is updated depending on the HMD position if necessary.
   * Only one billboard can be displayed at a time.
   */
  void ShowBillboard(std::string const& text, bool updatePosition, std::string const& tfile);

  /**
   * Hide the billboard.
   */
  void HideBillboard();

  /**
   * Update the current billboard.
   * Its position is updated depending on the HMD position if necessary.
   */
  void UpdateBillboard(bool updatePosition);

  ///@{
  /**
   * Display next image or cell image in billboard. Meant for usage with Imago.
   */
  void MoveToNextImage();
  void MoveToNextCell();
  ///@}

  ///@{
  /**
   * Add/remove crop planes and thick crop planes.
   */
  void AddACropPlane(double* origin, double* normal);
  void collabAddACropPlane(double* origin, double* normal);
  void collabRemoveAllCropPlanes();
  void collabUpdateCropPlane(int count, double* origin, double* normal);
  void AddAThickCrop(vtkTransform* t);
  void collabAddAThickCrop(vtkTransform* t);
  void collabRemoveAllThickCrops();
  void collabUpdateThickCrop(int count, double* matrix);
  void ShowCropPlanes(bool visible);
  ///@}

  /**
   * Translate all thick crop planes forward (true) or backward (false).
   */
  void MoveThickCrops(bool forward);

  ///@{
  /**
   * Set/get whether crop planes should snap to the axes.
   */
  void SetCropSnapping(int val);
  bool GetCropSnapping() { return this->CropSnapping; }
  ///@}

  /**
   * Get the number of crop planes.
   */
  size_t GetNumberOfCropPlanes() { return this->CropPlanes.size(); }

  /**
   * Project controller events onto crop planes so their full wireframe area
   * can be selected and manipulated with a distant ray.
   */
  bool HandleRemoteCropPlaneSelect(vtkEventDataDevice3D* edd, bool* touchingWidget = nullptr);
  bool UpdateRemoteCropPlaneInteraction(vtkEventDataDevice3D* edd);
  bool AdjustRemoteCropPlaneDistance(vtkEventDataDevice3D* edd);

  /**
   * Attach all active crop planes to a newly created or replaced render prop.
   */
  void ApplyClippingPlanes(vtkProp* prop);

  ///@{
  /**
   * Plane widget showing the plane of the clip or slice filter driven from the
   * XR menu. It can be dragged with the controller ray like crop planes.
   * `size` is the edge length of the displayed plane, in world units.
   * The callback is invoked while the plane is dragged in XR, with `finished`
   * set once the drag ends.
   */
  using SectionPlaneCallback =
    std::function<void(const double origin[3], const double normal[3], bool finished)>;
  void ShowSectionPlane(const double origin[3], const double normal[3], double size);
  void HideSectionPlane();
  void SetSectionPlaneCallback(SectionPlaneCallback callback);
  ///@}

  /**
   * Get the number of thick crop planes.
   */
  size_t GetNumberOfThickCrops() { return this->ThickCrops.size(); }

  ///@{
  /**
   * Set/get the initial thickness in world coordinates for
   * thick crop planes. 0 indicates automatic setting.
   * Default is 0.
   */
  vtkSetMacro(DefaultCropThickness, double);
  vtkGetMacro(DefaultCropThickness, double);
  ///@}

  ///@{
  /**
   * Set/get the name of the cell data array to edit in XR.
   */
  vtkSetMacro(EditableField, std::string);
  vtkGetMacro(EditableField, std::string);
  ///@}

  /**
   * Set given value on the selected cell for the cell array defined by member EditableField.
   */
  void SetEditableFieldValue(std::string value);

  /**
   * Process pick event based on given selection.
   */
  void HandlePickEvent(vtkObject* caller, void* calldata);

  /**
   * Set last event data.
   */
  void SetLastEventData(vtkEventData* edd);

  /**
   * Set helper class.
   */
  void SetHelper(vtkPVXRInterfaceHelper* val);

  /**
   * Write any widget state that needs to be in the location state.
   */
  void SaveLocationState(vtkPVXRInterfaceHelperLocation& sd);

  /**
   * Release any graphics resources.
   */
  void ReleaseGraphicsResources();

  ///@{
  /**
   * Imago related methods.
   */
  bool LoginToImago(std::string const& uid, std::string const& pw);
  void SetImagoWorkspace(std::string val);
  void SetImagoDataset(std::string val);
  void SetImagoImageryType(std::string val);
  void SetImagoImageType(std::string val);
  void GetImagoWorkspaces(std::vector<std::string>& vals);
  void GetImagoDatasets(std::vector<std::string>& vals);
  void GetImagoImageryTypes(std::vector<std::string>& vals);
  void GetImagoImageTypes(std::vector<std::string>& vals);
  ///@}

  /**
   * Copy new plane widgets from ParaView into XR so they can be manipulated.
   */
  void UpdateWidgetsFromParaView();

  /**
   * Perform any cleanup required when quitting XR.
   */
  void Quit();

protected:
  vtkPVXRInterfaceWidgets();
  ~vtkPVXRInterfaceWidgets() override;

  bool HasCellImage(vtkStringArray* sa, vtkIdType currCell);
  bool FindCellImage(vtkDataSetAttributes* celld, vtkIdType currCell, std::string& image);
  bool IsCellImageDifferent(std::string const& oldimg, std::string const& newimg);
  bool EventCallback(vtkObject* object, unsigned long event, void* calldata);
  void UpdateTexture();

  /**
   * Plane widgets that can be dragged with the controller ray.
   */
  std::vector<vtkImplicitPlaneWidget2*> GetRayDraggablePlanes();

  void SectionPlaneEvent(vtkObject* caller, unsigned long event, void* calldata);

private:
  vtkPVXRInterfaceWidgets(const vtkPVXRInterfaceWidgets&) = delete;
  void operator=(const vtkPVXRInterfaceWidgets&) = delete;

  bool CropSnapping = false;
  bool WaitingForImage = false;
  double DefaultCropThickness = 0;
  unsigned long RenderObserver = 0;
  std::string EditableField;

  std::vector<vtkImplicitPlaneWidget2*> CropPlanes;
  std::vector<vtkBoxWidget2*> ThickCrops;
  std::future<vtkImageData*> ImageFuture;
  vtkSmartPointer<vtkEventData> LastEventData;
  vtkPVXRInterfaceHelper* Helper = nullptr;

  struct vtkInternals;
  std::unique_ptr<vtkInternals> Internals;
};

#endif
