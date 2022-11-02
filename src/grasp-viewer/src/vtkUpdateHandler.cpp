/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <viewer.h>

using namespace viewer;


vtkUpdateHandler* vtkUpdateHandler::New()
{
    return new vtkUpdateHandler;
}


void vtkUpdateHandler::SetViewer(Viewer* viewer)
{
    this->viewer = viewer;
}


void vtkUpdateHandler::Execute(vtkObject *caller, unsigned long vtkNotUsed(eventId), void * vtkNotUsed(callData))
{
    vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::SafeDownCast(caller);

    if (shutdown)
    {
        iren->GetRenderWindow()->Finalize();
        iren->TerminateApp();
    }
    else
    {
        viewer->update();
        iren->GetRenderWindow()->Render();
    }
}

void vtkUpdateHandler::ShutDown()
{
    shutdown = true;
}
