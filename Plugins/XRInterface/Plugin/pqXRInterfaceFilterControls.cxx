// SPDX-FileCopyrightText: Copyright (c) Kitware Inc.
// SPDX-License-Identifier: BSD-3-Clause
#include "pqXRInterfaceFilterControls.h"

#include "pqActiveObjects.h"
#include "pqApplicationCore.h"
#include "pqPipelineSource.h"
#include "pqServerManagerModel.h"
#include "pqView.h"
#include "vtkCompositeRepresentation.h"
#include "vtkDataObject.h"
#include "vtkGeometryRepresentation.h"
#include "vtkMath.h"
#include "vtkMatrix4x4.h"
#include "vtkNew.h"
#include "vtkPVArrayInformation.h"
#include "vtkPVDataInformation.h"
#include "vtkPVDataSetAttributesInformation.h"
#include "vtkPVLODActor.h"
#include "vtkPVXRInterfaceHelper.h"
#include "vtkPVXRInterfaceWidgets.h"
#include "vtkSMParaViewPipelineControllerWithRendering.h"
#include "vtkSMPropertyHelper.h"
#include "vtkSMRepresentationProxy.h"
#include "vtkSMSessionProxyManager.h"
#include "vtkSMSourceProxy.h"
#include "vtkSMViewProxy.h"
#include "vtkSmartPointer.h"
#include "vtkTransform.h"
#include "vtkWeakPointer.h"

#include <QComboBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPointer>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <map>
#include <string>
#include <vector>

namespace
{
enum SectionMode
{
  CLIP = 0,
  SLICE = 1
};

enum SectionAxis
{
  AXIS_X = 0,
  AXIS_Y,
  AXIS_Z,
  AXIS_SURFACE
};

constexpr const char* AxisNames[3] = { "X", "Y", "Z" };
constexpr int ContourSliderSteps = 1000;

struct SectionState
{
  vtkSmartPointer<vtkSMSourceProxy> Filter;
  int Mode = CLIP;
  bool Invert = true; // ParaView default for Clip
  int Axis = AXIS_X;
  double SurfaceNormal[3] = { 1.0, 0.0, 0.0 };
  double Anchor[3] = { 0.0, 0.0, 0.0 };
  bool HasAnchor = false;
  int Position = 0;   // percent of the data extent along the normal
  int Rotation1 = 0;  // degrees
  int Rotation2 = 0;  // degrees
  int PlaneSize = 100; // percent of the data diagonal
  bool ShowPlane = true;
};

struct ContourState
{
  vtkSmartPointer<vtkSMSourceProxy> Filter;
  std::string Array;
  int Value = ContourSliderSteps / 2; // position in the array range
};

struct TargetState
{
  vtkWeakPointer<vtkSMSourceProxy> Input;
  SectionState Section;
  ContourState Contour;
  bool InputHidden = false; // hidden by us when the clip/slice was created
};

QPushButton* makeButton(const QString& text, QWidget* parent, bool checkable = false)
{
  auto* button = new QPushButton(text, parent);
  button->setMinimumHeight(40);
  button->setCheckable(checkable);
  button->setFocusPolicy(Qt::NoFocus);
  return button;
}

QSlider* makeSlider(int minimum, int maximum, QWidget* parent)
{
  auto* slider = new QSlider(Qt::Horizontal, parent);
  slider->setRange(minimum, maximum);
  slider->setMinimumHeight(32);
  slider->setFocusPolicy(Qt::NoFocus);
  return slider;
}

bool getBounds(vtkSMSourceProxy* input, double bounds[6])
{
  vtkPVDataInformation* info = input ? input->GetDataInformation(0) : nullptr;
  if (!info)
  {
    return false;
  }
  info->GetBounds(bounds);
  return vtkMath::AreBoundsInitialized(bounds);
}

double diagonal(const double bounds[6])
{
  const double dx = bounds[1] - bounds[0];
  const double dy = bounds[3] - bounds[2];
  const double dz = bounds[5] - bounds[4];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * Plane origin and normal from the section state, in data coordinates.
 */
void computePlane(const SectionState& state, const double bounds[6], double origin[3],
  double normal[3])
{
  double base[3] = { 0.0, 0.0, 0.0 };
  double axis1[3] = { 0.0, 0.0, 0.0 };
  double axis2[3] = { 0.0, 0.0, 0.0 };
  if (state.Axis == AXIS_SURFACE)
  {
    std::copy(state.SurfaceNormal, state.SurfaceNormal + 3, base);
    if (vtkMath::Normalize(base) == 0.0)
    {
      base[0] = 1.0;
    }
    vtkMath::Perpendiculars(base, axis1, axis2, 0.0);
  }
  else
  {
    base[state.Axis] = 1.0;
    axis1[(state.Axis + 1) % 3] = 1.0;
    axis2[(state.Axis + 2) % 3] = 1.0;
  }

  vtkNew<vtkTransform> rotation;
  rotation->RotateWXYZ(state.Rotation2, axis2);
  rotation->RotateWXYZ(state.Rotation1, axis1);
  rotation->TransformNormal(base, normal);
  vtkMath::Normalize(normal);

  const double center[3] = { 0.5 * (bounds[0] + bounds[1]), 0.5 * (bounds[2] + bounds[3]),
    0.5 * (bounds[4] + bounds[5]) };
  const double* anchor = state.HasAnchor ? state.Anchor : center;

  // Extent of the data along the normal: from the center the slider sweeps
  // half of it each way, from a surface point it may need to cross all of it.
  double extent = std::abs(normal[0]) * (bounds[1] - bounds[0]) +
    std::abs(normal[1]) * (bounds[3] - bounds[2]) + std::abs(normal[2]) * (bounds[5] - bounds[4]);
  if (!state.HasAnchor)
  {
    extent *= 0.5;
  }

  const double offset = extent * state.Position / 100.0;
  for (int i = 0; i < 3; ++i)
  {
    origin[i] = anchor[i] + offset * normal[i];
  }
}

/**
 * Matrix placing the data of `source` in the world. It is the identity unless
 * the prop was moved in XR (Grab) or given a display transform.
 */
void dataToWorld(vtkSMSourceProxy* source, vtkSMViewProxy* view, vtkMatrix4x4* matrix)
{
  matrix->Identity();
  vtkSMRepresentationProxy* repr = source && view ? view->FindRepresentation(source, 0) : nullptr;
  auto* composite =
    repr ? vtkCompositeRepresentation::SafeDownCast(repr->GetClientSideObject()) : nullptr;
  auto* geometry = vtkGeometryRepresentation::SafeDownCast(
    composite ? composite->GetActiveRepresentation() : nullptr);
  if (geometry && geometry->GetActor())
  {
    matrix->DeepCopy(geometry->GetActor()->GetMatrix());
  }
}

/**
 * Transform a plane with `matrix`. Normals use the inverse transpose.
 */
void transformPlane(vtkMatrix4x4* matrix, const double origin[3], const double normal[3],
  double outOrigin[3], double outNormal[3])
{
  const double point[4] = { origin[0], origin[1], origin[2], 1.0 };
  double result[4];
  matrix->MultiplyPoint(point, result);
  std::copy(result, result + 3, outOrigin);

  vtkNew<vtkMatrix4x4> inverse;
  vtkMatrix4x4::Invert(matrix, inverse);
  vtkNew<vtkMatrix4x4> normalMatrix;
  vtkMatrix4x4::Transpose(inverse, normalMatrix);
  const double direction[4] = { normal[0], normal[1], normal[2], 0.0 };
  normalMatrix->MultiplyPoint(direction, result);
  std::copy(result, result + 3, outNormal);
  if (vtkMath::Normalize(outNormal) == 0.0)
  {
    std::copy(normal, normal + 3, outNormal);
  }
}

std::vector<std::string> pointScalarArrays(vtkSMSourceProxy* input)
{
  std::vector<std::string> names;
  vtkPVDataInformation* info = input ? input->GetDataInformation(0) : nullptr;
  vtkPVDataSetAttributesInformation* pointData = info ? info->GetPointDataInformation() : nullptr;
  if (!pointData)
  {
    return names;
  }
  for (int i = 0; i < pointData->GetNumberOfArrays(); ++i)
  {
    vtkPVArrayInformation* array = pointData->GetArrayInformation(i);
    if (array && array->GetName() && array->GetNumberOfComponents() == 1)
    {
      names.emplace_back(array->GetName());
    }
  }
  return names;
}

bool arrayRange(vtkSMSourceProxy* input, const std::string& name, double range[2])
{
  vtkPVDataInformation* info = input ? input->GetDataInformation(0) : nullptr;
  vtkPVDataSetAttributesInformation* pointData = info ? info->GetPointDataInformation() : nullptr;
  vtkPVArrayInformation* array = pointData ? pointData->GetArrayInformation(name.c_str()) : nullptr;
  if (!array)
  {
    return false;
  }
  array->GetComponentRange(0, range);
  return range[0] <= range[1];
}

/**
 * Create a filter the same way the Properties panel Apply does, but without
 * leaving it in the "modified" state: the pipeline source is registered
 * as already applied.
 */
vtkSmartPointer<vtkSMSourceProxy> createFilter(const char* name, vtkSMSourceProxy* input)
{
  vtkSMSessionProxyManager* pxm = input->GetSessionProxyManager();
  vtkSmartPointer<vtkSMProxy> proxy;
  proxy.TakeReference(pxm->NewProxy("filters", name));
  vtkSmartPointer<vtkSMSourceProxy> filter = vtkSMSourceProxy::SafeDownCast(proxy);
  if (!filter)
  {
    return nullptr;
  }

  vtkNew<vtkSMParaViewPipelineControllerWithRendering> controller;
  controller->PreInitializeProxy(filter);
  vtkSMPropertyHelper(filter, "Input").Set(input, 0);
  controller->PostInitializeProxy(filter);
  filter->UpdateVTKObjects();
  controller->RegisterPipelineProxy(filter);
  return filter;
}

/**
 * Same rule as pqApplyBehavior for filters with replace_input="2": hide the
 * input when it is shown as an opaque surface.
 */
bool hideInputLikeParaView(vtkSMSourceProxy* input, vtkSMViewProxy* view)
{
  vtkSMRepresentationProxy* repr = view ? view->FindRepresentation(input, 0) : nullptr;
  if (!repr || vtkSMPropertyHelper(repr, "Visibility", /*quiet=*/true).GetAsInt() == 0)
  {
    return false;
  }
  const char* type = vtkSMPropertyHelper(repr, "Representation", /*quiet=*/true).GetAsString();
  const std::string reprType = type ? type : "";
  const double opacity = vtkSMPropertyHelper(repr, "Opacity", /*quiet=*/true).GetAsDouble();
  if ((reprType != "Surface" && reprType != "Surface With Edges") ||
    (opacity != 0.0 && opacity < 1.0))
  {
    return false;
  }
  vtkNew<vtkSMParaViewPipelineControllerWithRendering> controller;
  controller->Hide(input, 0, view);
  return true;
}

/**
 * Delete a filter created here, unless the user built more pipeline on top
 * of it in the meantime.
 */
void removeFilter(vtkSMSourceProxy* filter)
{
  for (unsigned int i = 0; i < filter->GetNumberOfConsumers(); ++i)
  {
    vtkSMProxy* consumer = filter->GetConsumerProxy(i);
    if (vtkSMSourceProxy::SafeDownCast(consumer) &&
      !vtkSMRepresentationProxy::SafeDownCast(consumer))
    {
      return;
    }
  }
  vtkNew<vtkSMParaViewPipelineController> controller;
  controller->UnRegisterProxy(filter);
}
}

//------------------------------------------------------------------------------
struct pqXRInterfaceFilterControls::pqInternals
{
  vtkPVXRInterfaceHelper* Helper = nullptr;

  QWidget* ClipPage = nullptr;
  QLabel* ClipTarget = nullptr;
  QPushButton* ClipButton = nullptr;
  QPushButton* SliceButton = nullptr;
  QPushButton* InvertButton = nullptr;
  QPushButton* AxisButtons[3] = { nullptr, nullptr, nullptr };
  QPushButton* AlignButton = nullptr;
  QLabel* PositionLabel = nullptr;
  QSlider* PositionSlider = nullptr;
  QLabel* Rotation1Label = nullptr;
  QSlider* Rotation1Slider = nullptr;
  QLabel* Rotation2Label = nullptr;
  QSlider* Rotation2Slider = nullptr;
  QLabel* SizeLabel = nullptr;
  QSlider* SizeSlider = nullptr;
  QPushButton* ShowPlaneButton = nullptr;
  QPushButton* ResetButton = nullptr;

  QWidget* ContourPage = nullptr;
  QLabel* ContourTarget = nullptr;
  QPushButton* ContourButton = nullptr;
  QComboBox* ArrayCombo = nullptr;
  QLabel* ValueLabel = nullptr;
  QSlider* ValueSlider = nullptr;
  QLabel* ContourInfo = nullptr;

  std::map<vtkSMSourceProxy*, TargetState> Targets;
  vtkWeakPointer<vtkSMSourceProxy> Current;
  bool PickPending = false;

  TargetState* current()
  {
    if (!this->Current)
    {
      return nullptr;
    }
    auto it = this->Targets.find(this->Current);
    return it == this->Targets.end() ? nullptr : &it->second;
  }

  /**
   * Data to world matrix of what shows the section: the clip/slice output
   * when it exists, the input otherwise.
   */
  void sectionToWorld(TargetState* target, vtkMatrix4x4* matrix) const
  {
    dataToWorld(target->Section.Filter ? target->Section.Filter.GetPointer()
                                       : target->Input.GetPointer(),
      this->view(), matrix);
  }

  vtkSMViewProxy* view() const
  {
    if (vtkSMViewProxy* view = this->Helper->GetSMView())
    {
      return view;
    }
    pqView* view = pqActiveObjects::instance().activeView();
    return view ? view->getViewProxy() : nullptr;
  }
};

//------------------------------------------------------------------------------
pqXRInterfaceFilterControls::pqXRInterfaceFilterControls(
  vtkPVXRInterfaceHelper* helper, QWidget* parent)
  : Superclass(parent)
  , Internals(new pqInternals())
{
  this->Internals->Helper = helper;
  this->Internals->ClipPage = new QWidget(parent);
  this->Internals->ContourPage = new QWidget(parent);
  this->buildClipPage();
  this->buildContourPage();

  // The helper is destroyed before this object (see pqXRInterfaceDockPanel),
  // so callbacks given to it guard against this object being gone instead.
  QPointer<pqXRInterfaceFilterControls> self(this);
  helper->GetWidgets()->SetSectionPlaneCallback(
    [self](const double origin[3], const double normal[3], bool finished)
    {
      if (self)
      {
        self->sectionPlaneDragged(origin, normal, finished);
      }
    });

  pqServerManagerModel* model = pqApplicationCore::instance()->getServerManagerModel();
  QObject::connect(model, &pqServerManagerModel::preSourceRemoved, this,
    &pqXRInterfaceFilterControls::sourceRemoved);

  this->setSelectedSource(nullptr);
}

//------------------------------------------------------------------------------
pqXRInterfaceFilterControls::~pqXRInterfaceFilterControls() = default;

//------------------------------------------------------------------------------
QWidget* pqXRInterfaceFilterControls::clipPage() const
{
  return this->Internals->ClipPage;
}

//------------------------------------------------------------------------------
QWidget* pqXRInterfaceFilterControls::contourPage() const
{
  return this->Internals->ContourPage;
}

//------------------------------------------------------------------------------
void pqXRInterfaceFilterControls::buildClipPage()
{
  auto& internals = *this->Internals;
  QWidget* page = internals.ClipPage;
  auto* layout = new QVBoxLayout(page);
  layout->setContentsMargins(8, 4, 8, 4);
  layout->setSpacing(6);

  internals.ClipTarget = new QLabel(page);
  internals.ClipTarget->setAlignment(Qt::AlignCenter);
  layout->addWidget(internals.ClipTarget);

  // Filter type. Clicking the active type again removes the filter.
  auto* modeRow = new QHBoxLayout();
  internals.ClipButton = makeButton(tr("Clip"), page, true);
  internals.SliceButton = makeButton(tr("Slice"), page, true);
  internals.InvertButton = makeButton(tr("Invert"), page, true);
  modeRow->addWidget(internals.ClipButton);
  modeRow->addWidget(internals.SliceButton);
  modeRow->addWidget(internals.InvertButton);
  layout->addLayout(modeRow);

  // Plane normal
  auto* axisRow = new QHBoxLayout();
  for (int axis = 0; axis < 3; ++axis)
  {
    internals.AxisButtons[axis] = makeButton(AxisNames[axis], page, true);
    axisRow->addWidget(internals.AxisButtons[axis]);
    QObject::connect(internals.AxisButtons[axis], &QPushButton::clicked, this,
      [this, axis]() { this->setSectionAxis(axis); });
  }
  internals.AlignButton = makeButton(tr("Align to Surface"), page, true);
  axisRow->addWidget(internals.AlignButton, 2);
  layout->addLayout(axisRow);

  // Fine tuning
  auto* grid = new QGridLayout();
  grid->setColumnStretch(1, 1);
  grid->setVerticalSpacing(2);
  const auto addSliderRow = [&](int row, QLabel*& label, QSlider*& slider, int min, int max)
  {
    label = new QLabel(page);
    label->setMinimumWidth(170);
    slider = makeSlider(min, max, page);
    grid->addWidget(label, row, 0);
    grid->addWidget(slider, row, 1);
  };
  addSliderRow(0, internals.PositionLabel, internals.PositionSlider, -100, 100);
  addSliderRow(1, internals.Rotation1Label, internals.Rotation1Slider, -90, 90);
  addSliderRow(2, internals.Rotation2Label, internals.Rotation2Slider, -90, 90);
  addSliderRow(3, internals.SizeLabel, internals.SizeSlider, 10, 200);
  layout->addLayout(grid);

  auto* bottomRow = new QHBoxLayout();
  internals.ShowPlaneButton = makeButton(tr("Show Plane"), page, true);
  internals.ResetButton = makeButton(tr("Reset Plane"), page);
  bottomRow->addWidget(internals.ShowPlaneButton);
  bottomRow->addWidget(internals.ResetButton);
  layout->addLayout(bottomRow);
  layout->addStretch(1);

  QObject::connect(internals.ClipButton, &QPushButton::clicked, this,
    [this]() { this->setSectionMode(CLIP); });
  QObject::connect(internals.SliceButton, &QPushButton::clicked, this,
    [this]() { this->setSectionMode(SLICE); });
  QObject::connect(internals.InvertButton, &QPushButton::toggled, this,
    [this](bool checked)
    {
      if (auto* target = this->Internals->current())
      {
        target->Section.Invert = checked;
        this->applySection();
      }
    });
  QObject::connect(internals.AlignButton, &QPushButton::clicked, this,
    &pqXRInterfaceFilterControls::alignSectionToSurface);
  QObject::connect(internals.ResetButton, &QPushButton::clicked, this,
    &pqXRInterfaceFilterControls::resetSection);
  QObject::connect(internals.ShowPlaneButton, &QPushButton::toggled, this,
    [this](bool checked)
    {
      if (auto* target = this->Internals->current())
      {
        target->Section.ShowPlane = checked;
        this->updateXRWidgets();
      }
    });

  const auto connectSlider = [this](QSlider* slider, int SectionState::*member)
  {
    QObject::connect(slider, &QSlider::valueChanged, this,
      [this, member](int value)
      {
        if (auto* target = this->Internals->current())
        {
          target->Section.*member = value;
          this->updateUI();
          this->applySection();
        }
      });
  };
  connectSlider(internals.PositionSlider, &SectionState::Position);
  connectSlider(internals.Rotation1Slider, &SectionState::Rotation1);
  connectSlider(internals.Rotation2Slider, &SectionState::Rotation2);
  connectSlider(internals.SizeSlider, &SectionState::PlaneSize);
}

//------------------------------------------------------------------------------
void pqXRInterfaceFilterControls::buildContourPage()
{
  auto& internals = *this->Internals;
  QWidget* page = internals.ContourPage;
  auto* layout = new QVBoxLayout(page);
  layout->setContentsMargins(8, 4, 8, 4);
  layout->setSpacing(6);

  internals.ContourTarget = new QLabel(page);
  internals.ContourTarget->setAlignment(Qt::AlignCenter);
  layout->addWidget(internals.ContourTarget);

  internals.ContourButton = makeButton(tr("Iso Surface"), page, true);
  layout->addWidget(internals.ContourButton);

  auto* grid = new QGridLayout();
  grid->setColumnStretch(1, 1);
  auto* arrayLabel = new QLabel(tr("Contour By"), page);
  internals.ArrayCombo = new QComboBox(page);
  internals.ArrayCombo->setMinimumHeight(40);
  internals.ArrayCombo->setFocusPolicy(Qt::NoFocus);
  grid->addWidget(arrayLabel, 0, 0);
  grid->addWidget(internals.ArrayCombo, 0, 1);
  internals.ValueLabel = new QLabel(page);
  internals.ValueLabel->setMinimumWidth(170);
  internals.ValueSlider = makeSlider(0, ContourSliderSteps, page);
  grid->addWidget(internals.ValueLabel, 1, 0);
  grid->addWidget(internals.ValueSlider, 1, 1);
  layout->addLayout(grid);

  internals.ContourInfo = new QLabel(page);
  internals.ContourInfo->setWordWrap(true);
  internals.ContourInfo->setAlignment(Qt::AlignCenter);
  layout->addWidget(internals.ContourInfo);
  layout->addStretch(1);

  QObject::connect(internals.ContourButton, &QPushButton::clicked, this,
    &pqXRInterfaceFilterControls::setContourEnabled);
  QObject::connect(internals.ArrayCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, &pqXRInterfaceFilterControls::setContourArray);
  QObject::connect(internals.ValueSlider, &QSlider::valueChanged, this,
    [this](int value)
    {
      if (auto* target = this->Internals->current())
      {
        target->Contour.Value = value;
        this->updateUI();
        this->applyContour();
      }
    });
}

//------------------------------------------------------------------------------
void pqXRInterfaceFilterControls::setSelectedSource(pqPipelineSource* source)
{
  if (!source)
  {
    source = pqActiveObjects::instance().activeSource();
  }
  vtkSMSourceProxy* proxy = source ? source->getSourceProxy() : nullptr;

  // Selecting one of our filters acts on its input
  for (auto& item : this->Internals->Targets)
  {
    if (proxy && (item.second.Section.Filter == proxy || item.second.Contour.Filter == proxy))
    {
      proxy = item.first;
      break;
    }
  }

  if (proxy && this->Internals->Targets.find(proxy) == this->Internals->Targets.end())
  {
    this->Internals->Targets[proxy].Input = proxy;
  }
  this->Internals->Current = proxy;

  this->updateUI();
  this->updateXRWidgets();
}

//------------------------------------------------------------------------------
void pqXRInterfaceFilterControls::setSectionMode(int mode)
{
  TargetState* target = this->Internals->current();
  vtkSMViewProxy* view = this->Internals->view();
  if (!target || !target->Input || !view)
  {
    this->updateUI();
    return;
  }

  SectionState& section = target->Section;
  const bool wasActive = section.Filter != nullptr;
  const bool sameMode = section.Mode == mode;
  if (wasActive)
  {
    vtkSmartPointer<vtkSMSourceProxy> filter = section.Filter;
    section.Filter = nullptr;
    // Unregistering may change the pipeline browser selection, keep our target
    vtkSMSourceProxy* input = target->Input;
    removeFilter(filter);
    this->Internals->Current = input;
  }
  section.Mode = mode;

  if (!(wasActive && sameMode))
  {
    section.Filter = createFilter(mode == CLIP ? "Clip" : "Cut", target->Input);
    if (section.Filter)
    {
      this->applySection();
      vtkNew<vtkSMParaViewPipelineControllerWithRendering> controller;
      controller->Show(section.Filter, 0, view);
      if (!target->InputHidden)
      {
        target->InputHidden = hideInputLikeParaView(target->Input, view);
      }
    }
  }
  else if (target->InputHidden)
  {
    // Turned off: bring back the original data
    vtkNew<vtkSMParaViewPipelineControllerWithRendering> controller;
    controller->Show(target->Input, 0, view);
    target->InputHidden = false;
  }

  this->updateUI();
  this->updateXRWidgets();
  this->render();
}

//------------------------------------------------------------------------------
void pqXRInterfaceFilterControls::setSectionAxis(int axis)
{
  TargetState* target = this->Internals->current();
  if (!target)
  {
    this->updateUI();
    return;
  }

  SectionState& section = target->Section;
  section.Axis = axis;
  section.HasAnchor = false;
  section.Position = 0;
  section.Rotation1 = 0;
  section.Rotation2 = 0;

  if (!section.Filter)
  {
    // Choosing a normal is enough to start clipping
    this->setSectionMode(section.Mode);
    return;
  }
  this->updateUI();
  this->applySection();
}

//------------------------------------------------------------------------------
void pqXRInterfaceFilterControls::alignSectionToSurface()
{
  if (!this->Internals->current())
  {
    this->updateUI();
    return;
  }

  // Clicking again cancels
  this->Internals->PickPending = !this->Internals->PickPending;
  if (!this->Internals->PickPending)
  {
    this->Internals->Helper->RequestSurfacePick(nullptr);
    this->updateUI();
    return;
  }

  QPointer<pqXRInterfaceFilterControls> self(this);
  this->Internals->Helper->RequestSurfacePick(
    [self](bool hit, const double position[3], const double normal[3])
    {
      if (!self)
      {
        return;
      }
      const std::array<double, 3> pos = { position[0], position[1], position[2] };
      const std::array<double, 3> dir = { normal[0], normal[1], normal[2] };
      // Leave the interactor callback before touching the pipeline
      QMetaObject::invokeMethod(
        self.data(),
        [self, hit, pos, dir]()
        {
          if (!self)
          {
            return;
          }
          self->Internals->PickPending = false;
          TargetState* target = self->Internals->current();
          if (!hit || !target)
          {
            self->updateUI();
            return;
          }

          // Picks are in world coordinates, the section in data coordinates
          vtkNew<vtkMatrix4x4> toWorld;
          self->Internals->sectionToWorld(target, toWorld);
          vtkNew<vtkMatrix4x4> toData;
          vtkMatrix4x4::Invert(toWorld, toData);

          SectionState& section = target->Section;
          section.Axis = AXIS_SURFACE;
          transformPlane(toData, pos.data(), dir.data(), section.Anchor, section.SurfaceNormal);
          section.HasAnchor = true;
          section.Position = 0;
          section.Rotation1 = 0;
          section.Rotation2 = 0;
          if (!section.Filter)
          {
            self->setSectionMode(section.Mode);
            return;
          }
          self->updateUI();
          self->applySection();
        },
        Qt::QueuedConnection);
    });
  this->updateUI();
}

//------------------------------------------------------------------------------
void pqXRInterfaceFilterControls::resetSection()
{
  TargetState* target = this->Internals->current();
  if (!target)
  {
    return;
  }
  SectionState& section = target->Section;
  if (section.Axis == AXIS_SURFACE)
  {
    section.Axis = AXIS_X;
  }
  section.HasAnchor = false;
  section.Position = 0;
  section.Rotation1 = 0;
  section.Rotation2 = 0;
  section.PlaneSize = 100;
  this->updateUI();
  this->applySection();
}

//------------------------------------------------------------------------------
void pqXRInterfaceFilterControls::applySection()
{
  TargetState* target = this->Internals->current();
  double bounds[6];
  if (!target || !target->Section.Filter || !getBounds(target->Input, bounds))
  {
    this->updateXRWidgets();
    return;
  }

  SectionState& section = target->Section;
  double origin[3];
  double normal[3];
  computePlane(section, bounds, origin, normal);

  vtkSMProxy* plane =
    vtkSMPropertyHelper(section.Filter, section.Mode == CLIP ? "ClipFunction" : "CutFunction")
      .GetAsProxy();
  if (plane)
  {
    vtkSMPropertyHelper(plane, "Origin").Set(origin, 3);
    vtkSMPropertyHelper(plane, "Normal").Set(normal, 3);
    plane->UpdateVTKObjects();
  }
  if (section.Mode == CLIP)
  {
    vtkSMPropertyHelper(section.Filter, "Invert").Set(section.Invert ? 1 : 0);
  }
  section.Filter->UpdateVTKObjects();

  this->updateXRWidgets();
  this->render();
}

//------------------------------------------------------------------------------
void pqXRInterfaceFilterControls::sectionPlaneDragged(
  const double origin[3], const double normal[3], bool finished)
{
  TargetState* target = this->Internals->current();
  if (!target || !target->Section.Filter)
  {
    return;
  }

  // The widget lives in world coordinates, the filter in data coordinates
  vtkNew<vtkMatrix4x4> toWorld;
  this->Internals->sectionToWorld(target, toWorld);
  vtkNew<vtkMatrix4x4> toData;
  vtkMatrix4x4::Invert(toWorld, toData);
  double dataOrigin[3];
  double dataNormal[3];
  transformPlane(toData, origin, normal, dataOrigin, dataNormal);

  SectionState& section = target->Section;
  vtkSMProxy* plane =
    vtkSMPropertyHelper(section.Filter, section.Mode == CLIP ? "ClipFunction" : "CutFunction")
      .GetAsProxy();
  if (plane)
  {
    vtkSMPropertyHelper(plane, "Origin").Set(dataOrigin, 3);
    vtkSMPropertyHelper(plane, "Normal").Set(dataNormal, 3);
    plane->UpdateVTKObjects();
    this->render();
  }

  if (finished)
  {
    // The dragged plane becomes the new reference for the sliders
    section.Axis = AXIS_SURFACE;
    std::copy(dataNormal, dataNormal + 3, section.SurfaceNormal);
    std::copy(dataOrigin, dataOrigin + 3, section.Anchor);
    section.HasAnchor = true;
    section.Position = 0;
    section.Rotation1 = 0;
    section.Rotation2 = 0;
    this->updateUI();
  }
}

//------------------------------------------------------------------------------
void pqXRInterfaceFilterControls::setContourEnabled(bool enabled)
{
  TargetState* target = this->Internals->current();
  vtkSMViewProxy* view = this->Internals->view();
  if (!target || !target->Input || !view)
  {
    this->updateUI();
    return;
  }

  ContourState& contour = target->Contour;
  if (enabled && !contour.Filter)
  {
    const std::vector<std::string> arrays = pointScalarArrays(target->Input);
    if (arrays.empty())
    {
      this->updateUI();
      return;
    }
    if (std::find(arrays.begin(), arrays.end(), contour.Array) == arrays.end())
    {
      contour.Array = arrays.front();
    }

    contour.Filter = createFilter("Contour", target->Input);
    if (contour.Filter)
    {
      vtkSMPropertyHelper(contour.Filter, "ComputeScalars", /*quiet=*/true).Set(1);
      this->applyContour();
      // Iso lines/surfaces are shown on top of the data, the input stays visible
      vtkNew<vtkSMParaViewPipelineControllerWithRendering> controller;
      controller->Show(contour.Filter, 0, view);
    }
  }
  else if (!enabled && contour.Filter)
  {
    vtkSmartPointer<vtkSMSourceProxy> filter = contour.Filter;
    contour.Filter = nullptr;
    // Unregistering may change the pipeline browser selection, keep our target
    vtkSMSourceProxy* input = target->Input;
    removeFilter(filter);
    this->Internals->Current = input;
  }

  this->updateUI();
  this->render();
}

//------------------------------------------------------------------------------
void pqXRInterfaceFilterControls::setContourArray(int index)
{
  TargetState* target = this->Internals->current();
  if (!target || index < 0)
  {
    return;
  }
  target->Contour.Array = this->Internals->ArrayCombo->itemText(index).toStdString();
  this->updateUI();
  this->applyContour();
}

//------------------------------------------------------------------------------
void pqXRInterfaceFilterControls::applyContour()
{
  TargetState* target = this->Internals->current();
  if (!target || !target->Contour.Filter)
  {
    return;
  }

  ContourState& contour = target->Contour;
  double range[2] = { 0.0, 0.0 };
  arrayRange(target->Input, contour.Array, range);
  const double value =
    range[0] + (range[1] - range[0]) * contour.Value / static_cast<double>(ContourSliderSteps);

  vtkSMPropertyHelper(contour.Filter, "SelectInputScalars")
    .SetInputArrayToProcess(vtkDataObject::FIELD_ASSOCIATION_POINTS, contour.Array.c_str());
  vtkSMPropertyHelper values(contour.Filter, "ContourValues");
  values.SetNumberOfElements(1);
  values.Set(0u, value);
  contour.Filter->UpdateVTKObjects();

  this->render();
}

//------------------------------------------------------------------------------
void pqXRInterfaceFilterControls::updateXRWidgets()
{
  vtkPVXRInterfaceWidgets* widgets = this->Internals->Helper->GetWidgets();
  TargetState* target = this->Internals->current();
  double bounds[6];
  if (!this->Internals->Helper->InVR() || !target || !target->Section.Filter ||
    !target->Section.ShowPlane || !getBounds(target->Input, bounds))
  {
    widgets->HideSectionPlane();
    return;
  }

  double origin[3];
  double normal[3];
  computePlane(target->Section, bounds, origin, normal);

  // Show the plane where the data is drawn
  vtkNew<vtkMatrix4x4> toWorld;
  this->Internals->sectionToWorld(target, toWorld);
  double worldOrigin[3];
  double worldNormal[3];
  transformPlane(toWorld, origin, normal, worldOrigin, worldNormal);
  const double scale = std::cbrt(std::abs(toWorld->Determinant()));
  widgets->ShowSectionPlane(worldOrigin, worldNormal,
    scale * diagonal(bounds) * target->Section.PlaneSize / 100.0);
}

//------------------------------------------------------------------------------
void pqXRInterfaceFilterControls::updateUI()
{
  auto& internals = *this->Internals;
  TargetState* target = internals.current();
  const bool hasTarget = target && target->Input;

  QString targetText = tr("Select data in the pipeline browser");
  if (hasTarget)
  {
    pqServerManagerModel* model = pqApplicationCore::instance()->getServerManagerModel();
    pqPipelineSource* source = model->findItem<pqPipelineSource*>(target->Input.GetPointer());
    targetText = source ? source->getSMName() : tr("Selected data");
  }
  internals.ClipTarget->setText(targetText);
  internals.ContourTarget->setText(targetText);

  // Clip page
  const SectionState defaults;
  const SectionState& section = hasTarget ? target->Section : defaults;
  const bool sectionActive = section.Filter != nullptr;
  {
    const QSignalBlocker b1(internals.ClipButton);
    const QSignalBlocker b2(internals.SliceButton);
    const QSignalBlocker b3(internals.InvertButton);
    const QSignalBlocker b4(internals.ShowPlaneButton);
    const QSignalBlocker b5(internals.PositionSlider);
    const QSignalBlocker b6(internals.Rotation1Slider);
    const QSignalBlocker b7(internals.Rotation2Slider);
    const QSignalBlocker b8(internals.SizeSlider);
    const QSignalBlocker b9(internals.AlignButton);

    internals.ClipButton->setChecked(sectionActive && section.Mode == CLIP);
    internals.SliceButton->setChecked(sectionActive && section.Mode == SLICE);
    internals.InvertButton->setChecked(section.Invert);
    for (int axis = 0; axis < 3; ++axis)
    {
      const QSignalBlocker blocker(internals.AxisButtons[axis]);
      internals.AxisButtons[axis]->setChecked(section.Axis == axis);
    }
    internals.AlignButton->setChecked(internals.PickPending || section.Axis == AXIS_SURFACE);
    internals.AlignButton->setText(
      internals.PickPending ? tr("Aim + Trigger...") : tr("Align to Surface"));
    internals.ShowPlaneButton->setChecked(section.ShowPlane);
    internals.PositionSlider->setValue(section.Position);
    internals.Rotation1Slider->setValue(section.Rotation1);
    internals.Rotation2Slider->setValue(section.Rotation2);
    internals.SizeSlider->setValue(section.PlaneSize);
  }

  internals.PositionLabel->setText(tr("Position: %1%").arg(section.Position));
  if (section.Axis == AXIS_SURFACE)
  {
    internals.Rotation1Label->setText(tr("Tilt 1: %1°").arg(section.Rotation1));
    internals.Rotation2Label->setText(tr("Tilt 2: %1°").arg(section.Rotation2));
  }
  else
  {
    internals.Rotation1Label->setText(
      tr("Rotate about %1: %2°").arg(AxisNames[(section.Axis + 1) % 3]).arg(section.Rotation1));
    internals.Rotation2Label->setText(
      tr("Rotate about %1: %2°").arg(AxisNames[(section.Axis + 2) % 3]).arg(section.Rotation2));
  }
  internals.SizeLabel->setText(tr("Plane Size: %1%").arg(section.PlaneSize));

  internals.ClipButton->setEnabled(hasTarget);
  internals.SliceButton->setEnabled(hasTarget);
  internals.InvertButton->setEnabled(hasTarget && section.Mode == CLIP);
  for (QPushButton* button : internals.AxisButtons)
  {
    button->setEnabled(hasTarget);
  }
  internals.AlignButton->setEnabled(hasTarget);
  for (QWidget* widget : { static_cast<QWidget*>(internals.PositionSlider),
         static_cast<QWidget*>(internals.Rotation1Slider),
         static_cast<QWidget*>(internals.Rotation2Slider),
         static_cast<QWidget*>(internals.SizeSlider),
         static_cast<QWidget*>(internals.ShowPlaneButton),
         static_cast<QWidget*>(internals.ResetButton) })
  {
    widget->setEnabled(hasTarget);
  }

  // Iso surface page
  const ContourState contourDefaults;
  const ContourState& contour = hasTarget ? target->Contour : contourDefaults;
  const std::vector<std::string> arrays =
    hasTarget ? pointScalarArrays(target->Input) : std::vector<std::string>();
  {
    const QSignalBlocker b1(internals.ContourButton);
    const QSignalBlocker b2(internals.ArrayCombo);
    const QSignalBlocker b3(internals.ValueSlider);

    internals.ContourButton->setChecked(contour.Filter != nullptr);
    internals.ArrayCombo->clear();
    for (const std::string& name : arrays)
    {
      internals.ArrayCombo->addItem(QString::fromStdString(name));
    }
    const auto it = std::find(arrays.begin(), arrays.end(), contour.Array);
    internals.ArrayCombo->setCurrentIndex(
      it == arrays.end() ? 0 : static_cast<int>(std::distance(arrays.begin(), it)));
    internals.ValueSlider->setValue(contour.Value);
  }

  const std::string arrayName = contour.Array.empty() && !arrays.empty() ? arrays.front()
                                                                          : contour.Array;
  double range[2] = { 0.0, 0.0 };
  if (hasTarget && arrayRange(target->Input, arrayName, range))
  {
    const double value =
      range[0] + (range[1] - range[0]) * contour.Value / static_cast<double>(ContourSliderSteps);
    internals.ValueLabel->setText(tr("Value: %1").arg(value, 0, 'g', 4));
  }
  else
  {
    internals.ValueLabel->setText(tr("Value"));
  }

  const bool canContour = hasTarget && !arrays.empty();
  internals.ContourButton->setEnabled(canContour);
  internals.ArrayCombo->setEnabled(canContour);
  internals.ValueSlider->setEnabled(canContour);
  internals.ContourInfo->setText(hasTarget && arrays.empty()
      ? tr("Iso surfaces need a point data array with one component.")
      : QString());
}

//------------------------------------------------------------------------------
void pqXRInterfaceFilterControls::render()
{
  if (this->Internals->Helper->InVR())
  {
    // the XR event loop renders the ParaView view and refreshes the props
    this->Internals->Helper->SetNeedStillRender(true);
  }
  else if (vtkSMViewProxy* view = this->Internals->view())
  {
    view->StillRender();
  }
}

//------------------------------------------------------------------------------
void pqXRInterfaceFilterControls::sourceRemoved(pqPipelineSource* source)
{
  vtkSMSourceProxy* proxy = source ? source->getSourceProxy() : nullptr;
  if (!proxy)
  {
    return;
  }

  auto& targets = this->Internals->Targets;
  auto it = targets.find(proxy);
  if (it != targets.end())
  {
    targets.erase(it);
  }
  else
  {
    for (auto& item : targets)
    {
      if (item.second.Section.Filter == proxy)
      {
        item.second.Section.Filter = nullptr;
      }
      if (item.second.Contour.Filter == proxy)
      {
        item.second.Contour.Filter = nullptr;
      }
    }
  }

  this->updateUI();
  this->updateXRWidgets();
}
