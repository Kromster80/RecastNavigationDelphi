program recastnavigation;
uses
  //FastMM4,
  Vcl.Forms,
  Unit_Form in 'Unit_Form.pas' {Form1},
  Unit_FrameCrowdTool in 'Unit_FrameCrowdTool.pas' {FrameCrowdTool: TFrame},
  Unit_FramePruneTool in 'Unit_FramePruneTool.pas' {FramePruneTool: TFrame},
  Unit_FrameTesterTool in 'Unit_FrameTesterTool.pas' {FrameTesterTool: TFrame},
  RN_ChunkyTriMesh in 'RN_ChunkyTriMesh.pas',
  RN_CrowdTool in 'RN_CrowdTool.pas',
  RN_DebugDraw in 'RN_DebugDraw.pas',
  RN_DetourCommon in 'RN_DetourCommon.pas',
  RN_DetourCrowd in 'RN_DetourCrowd.pas',
  RN_DetourDebugDraw in 'RN_DetourDebugDraw.pas',
  RN_DetourDump in 'RN_DetourDump.pas',
  RN_DetourLocalBoundary in 'RN_DetourLocalBoundary.pas',
  RN_DetourNavMesh in 'RN_DetourNavMesh.pas',
  RN_DetourNavMeshBuilder in 'RN_DetourNavMeshBuilder.pas',
  RN_DetourNavMeshHelper in 'RN_DetourNavMeshHelper.pas',
  RN_DetourNavMeshQuery in 'RN_DetourNavMeshQuery.pas',
  RN_DetourNode in 'RN_DetourNode.pas',
  RN_DetourObstacleAvoidance in 'RN_DetourObstacleAvoidance.pas',
  RN_DetourPathCorridor in 'RN_DetourPathCorridor.pas',
  RN_DetourPathQueue in 'RN_DetourPathQueue.pas',
  RN_DetourProximityGrid in 'RN_DetourProximityGrid.pas',
  RN_DetourStatus in 'RN_DetourStatus.pas',
  RN_Helper in 'RN_Helper.pas',
  RN_InputGeom in 'RN_InputGeom.pas',
  RN_MeshLoaderObj in 'RN_MeshLoaderObj.pas',
  RN_NavMeshPruneTool in 'RN_NavMeshPruneTool.pas',
  RN_NavMeshTesterTool in 'RN_NavMeshTesterTool.pas',
  RN_PerfTimer in 'RN_PerfTimer.pas',
  RN_Recast in 'RN_Recast.pas',
  RN_RecastAlloc in 'RN_RecastAlloc.pas',
  RN_RecastArea in 'RN_RecastArea.pas',
  RN_RecastContour in 'RN_RecastContour.pas',
  RN_RecastContourHelper in 'RN_RecastContourHelper.pas',
  RN_RecastDebugDraw in 'RN_RecastDebugDraw.pas',
  RN_RecastDump in 'RN_RecastDump.pas',
  RN_RecastFilter in 'RN_RecastFilter.pas',
  RN_RecastHelper in 'RN_RecastHelper.pas',
  RN_RecastLayers in 'RN_RecastLayers.pas',
  RN_RecastMesh in 'RN_RecastMesh.pas',
  RN_RecastMeshDetail in 'RN_RecastMeshDetail.pas',
  RN_RecastRasterization in 'RN_RecastRasterization.pas',
  RN_RecastRegion in 'RN_RecastRegion.pas',
  RN_Sample in 'RN_Sample.pas',
  RN_SampleInterfaces in 'RN_SampleInterfaces.pas',
  RN_SampleSoloMesh in 'RN_SampleSoloMesh.pas',
  RN_ValueHistory in 'RN_ValueHistory.pas';

{$R *.res}

begin
  Application.Initialize;
  Application.MainFormOnTaskbar := True;
  Application.CreateForm(TForm1, Form1);
  Application.Run;
end.
