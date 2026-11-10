// SPDX-FileCopyrightText: Copyright (c) Kitware Inc.
// SPDX-License-Identifier: BSD-3-Clause
/**
 * @class   pqXRInterfaceFilterControls
 * @brief   XR menu pages to clip/slice and contour the selected source
 *
 * Builds the "Clip" and "Iso Surface" pages of the XR menu. They drive the
 * standard ParaView "Clip", "Slice" and "Contour" filters on the selected
 * pipeline source. Filters are created, updated and removed directly through
 * the server manager so changes are visible immediately, without Apply.
 *
 * The clip/slice plane is defined by an axis (X, Y, Z or a surface normal
 * picked with the controller ray), a position along that normal and two
 * rotations. It is shown in XR as a plane widget that can also be dragged
 * with the controller ray.
 *
 * Instantiated by pqXRInterfaceControls.
 */

#ifndef pqXRInterfaceFilterControls_h
#define pqXRInterfaceFilterControls_h

#include <QObject>
#include <QScopedPointer>

class pqPipelineSource;
class QWidget;
class vtkPVXRInterfaceHelper;

class pqXRInterfaceFilterControls : public QObject
{
  Q_OBJECT
  typedef QObject Superclass;

public:
  pqXRInterfaceFilterControls(vtkPVXRInterfaceHelper* helper, QWidget* parent);
  ~pqXRInterfaceFilterControls() override;

  /**
   * Pages to add to the XR menu tab widget.
   */
  QWidget* clipPage() const;
  QWidget* contourPage() const;

public Q_SLOTS:
  /**
   * Set the source the pages act on. When the given source is a filter
   * created by these pages, its input is used instead. A null source falls
   * back to the active source.
   */
  void setSelectedSource(pqPipelineSource* source);

  /**
   * Show the plane widget of the current clip/slice in XR. Called when
   * entering XR as widgets do not survive between XR sessions.
   */
  void updateXRWidgets();

private:
  Q_DISABLE_COPY(pqXRInterfaceFilterControls)

  void buildClipPage();
  void buildContourPage();

  void setSectionMode(int mode);
  void setSectionAxis(int axis);
  void alignSectionToSurface();
  void resetSection();
  void applySection();
  void sectionPlaneDragged(const double origin[3], const double normal[3], bool finished);

  void setContourEnabled(bool enabled);
  void setContourArray(int index);
  void applyContour();

  void updateUI();
  void render();
  void sourceRemoved(pqPipelineSource* source);

  struct pqInternals;
  QScopedPointer<pqInternals> Internals;
};

#endif
