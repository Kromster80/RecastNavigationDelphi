unit Unit_Form;
interface
uses
  Windows, Messages, SysUtils, Classes, Graphics, Controls, ComCtrls, Forms, StrUtils,
  StdCtrls, ExtCtrls, CheckLst, Spin, Math, Dialogs, dglOpenGL, KromOGLUtils,
  RN_InputGeom, RN_Recast, RN_SampleInterfaces, RN_Sample, RN_SampleSoloMesh, RN_SampleTileMesh;

type
  TForm1 = class(TForm)
    Panel3: TPanel;
    Timer1: TTimer;
    Memo1: TMemo;
    gbSample: TGroupBox;
    rgTool: TRadioGroup;
    gbTool: TGroupBox;
    Panel1: TPanel;
    rgInputMesh: TRadioGroup;
    rgChooseSample: TRadioGroup;
    CheckBox1: TCheckBox;
    btnBuild: TButton;
    procedure FormCreate(Sender: TObject);
    procedure FormDestroy(Sender: TObject);
    procedure FormResize(Sender: TObject);
    procedure DoIdle(Sender: TObject; var Done: Boolean);
    procedure Panel3MouseDown(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Integer);
    procedure Panel3MouseMove(Sender: TObject; Shift: TShiftState; X, Y: Integer);
    procedure Timer1Timer(Sender: TObject);
    procedure FormMouseWheel(Sender: TObject; Shift: TShiftState; WheelDelta: Integer; MousePos: TPoint; var Handled: Boolean);
    procedure btnBuildClick(Sender: TObject);
    procedure btnToolClick(Sender: TObject);
    procedure rgChooseSampleClick(Sender: TObject);
  private
    h_DC: HDC;
    h_RC: HGLRC;
    fExeDir: string;
    fFrameTime: Single;
    fPrevX: Single;
    fPrevY: Single;
    fRotateX: Single;
    fRotateY: Single;
    fDist: Single;

    fRayS: array [0..2] of Single;
    fRayE: array [0..2] of Single;

    fGeom: TInputGeom;
    fSample: TSample;

    procedure InitGL;
    procedure UpdateModelViewProjection;
  end;

var
  Form1: TForm1;


implementation
uses RN_RecastDump;

{$R *.dfm}

function ScanObjFiles(aPath: string): string;
var
  SearchRec: TSearchRec;
begin
  Result := '';

  if not DirectoryExists(aPath) then Exit;

  FindFirst(aPath + '*.obj', faAnyFile - faDirectory, SearchRec);
  repeat
    Result := Result + ChangeFileExt(SearchRec.Name, '') + sLineBreak;
  until (FindNext(SearchRec) <> 0);
  FindClose(SearchRec);
end;


procedure TForm1.InitGL;
begin
  //Means it will receive WM_SIZE WM_PAINT always in pair (if False - WM_PAINT is not called if size becames smaller)
  Panel3.FullRepaint := True;

  SetRenderFrameAA(Panel1.Handle, Panel3.Handle, 16, h_DC, h_RC);

  //RenderArea.CreateRenderContext(True);
  glClearColor(0.75, 0.75, 0.8, 1);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); //Set alpha mode
  glEnable(GL_TEXTURE_2D);                     // Enable Texture Mapping

  glPolygonMode(GL_FRONT, GL_FILL);
  glEnable(GL_NORMALIZE);
  glHint(GL_GENERATE_MIPMAP_HINT, GL_NICEST);
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
  glEnable(GL_COLOR_MATERIAL);                 //Enable Materials
  glDisable(GL_LIGHTING); //We don't need it
  glEnable(GL_DEPTH_TEST);

  if WGL_EXT_swap_control then
    wglSwapIntervalEXT(0);

  FormResize(Self);
end;


procedure TForm1.FormCreate(Sender: TObject);
begin
  Set8087CW($133F);

  fExeDir := ExtractFilePath(Application.ExeName);

  rgInputMesh.Items.Text := ScanObjFiles(fExeDir);

  fDist := 100;
  fRotateY := -45;

  InitGL;

  fRotateX := 0;

  rgChooseSampleClick(nil);
  btnBuildClick(nil);

  Application.OnIdle := DoIdle;
end;


procedure TForm1.FormDestroy(Sender: TObject);
begin
  fGeom.Free;
  fSample.Free;
end;


procedure TForm1.FormMouseWheel(Sender: TObject; Shift: TShiftState; WheelDelta: Integer; MousePos: TPoint; var Handled: Boolean);
begin
  if Panel3.BoundsRect.Contains(ScreenToClient(MousePos)) then
    fDist := fDist - Sign(WheelDelta) * 5;

  UpdateModelViewProjection;

  Handled := True;
end;

procedure TForm1.FormResize(Sender: TObject);
begin
  glViewport(0, 0, Panel3.Width, Panel3.Height);

  UpdateModelViewProjection;
end;


procedure TForm1.rgChooseSampleClick(Sender: TObject);
begin
  FreeAndNil(fSample);

  case rgChooseSample.ItemIndex of
    0: fSample := TSample_SoloMesh.Create(gbSample, gbTool, rgTool);
    1: fSample := TSample_TileMesh.Create(gbSample, gbTool, rgTool);
  end;
end;

procedure TForm1.Panel3MouseDown(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Integer);
var hitt: Single; hit: Boolean; pos: array [0..2] of Single;
begin
  if (fGeom <> nil) and (fSample <> nil) then
  begin
    hit := fGeom.raycastMesh(@fRayS[0], @fRayE[0], @hitt);

    if (hit) then
    begin
      pos[0] := fRayS[0] + (fRayE[0] - fRayS[0])*hitt;
      pos[1] := fRayS[1] + (fRayE[1] - fRayS[1])*hitt;
      pos[2] := fRayS[2] + (fRayE[2] - fRayS[2])*hitt;
      fSample.handleClick(@fRayS[0], @pos[0], Button = mbRight);
    end;
  end;

  fPrevX := X;
  fPrevY := Y;
end;


procedure TForm1.Panel3MouseMove(Sender: TObject; Shift: TShiftState; X, Y: Integer);
var
  proj, model: TGLMatrixd4;
  view: TGLVectori4;
  dx,dy,dz: Double;
begin
  if ssLeft in Shift then
  begin
    fRotateX := fRotateX + (X - fPrevX);
    fRotateY := EnsureRange(fRotateY + (Y - fPrevY), -85, 85);

    UpdateModelViewProjection;

    fPrevX := X;
    fPrevY := Y;
  end;

  // Get hit ray position and direction.
  glGetDoublev(GL_PROJECTION_MATRIX, @proj[0,0]);
  glGetDoublev(GL_MODELVIEW_MATRIX, @model[0,0]);
  glGetIntegerv(GL_VIEWPORT, @view[0]);
  gluUnProject(X, Panel3.Height - Y, 0.0, model, proj, view, @dx, @dy, @dz);
  fRayS[0] := dx; fRayS[1] := dy; fRayS[2] := dz;
  gluUnProject(X, Panel3.Height - Y, 1.0, model, proj, view, @dx, @dy, @dz);
  fRayE[0] := dx; fRayE[1] := dy; fRayE[2] := dz;
end;


procedure TForm1.btnBuildClick(Sender: TObject);
var
  ctx: TBuildContext;
  meshName: string;
  I: Integer;
begin
  ctx := TBuildContext.Create;

  meshName := fExeDir + rgInputMesh.Items[rgInputMesh.ItemIndex] + '.obj';
  fGeom := TInputGeom.Create;
  fGeom.loadMesh(ctx, meshName);

  fSample.setContext := ctx;

  fSample.handleMeshChanged(fGeom);
  fSample.handleSettings;
  ctx.resetLog();
  if fSample.handleBuild() then
    ctx.dumpLog(Format('Build log %s: ', [meshName]));

  for I := 0 to ctx.getLogCount - 1 do
    Memo1.Lines.Append(ctx.getLogText(I));

  ctx.Free;
end;

procedure TForm1.btnToolClick(Sender: TObject);
begin
  fSample.handleMenu(Sender);
end;

procedure TForm1.DoIdle(Sender: TObject; var Done: Boolean);
var
  prevTime: Int64;
  freq: Int64;
  newTime: Int64;
begin
  QueryPerformanceCounter(prevTime);

  glClear(GL_COLOR_BUFFER_BIT or GL_DEPTH_BUFFER_BIT);

  // Axis
  glLineWidth(2);
  glBegin(GL_LINES);
    glColor4f(1,0,0,1); glVertex3f(0,0,0); glVertex3f(1,0,0);
    glColor4f(0,1,0,1); glVertex3f(0,0,0); glVertex3f(0,1,0);
    glColor4f(0,0,1,1); glVertex3f(0,0,0); glVertex3f(0,0,1);
  glEnd;

  if fSample <> nil then
    fSample.handleRender;

  SwapBuffers(h_DC);

  QueryPerformanceFrequency(freq);
  QueryPerformanceCounter(newTime);
  fFrameTime := (newTime - prevTime) / (freq / 1000);

  Done := False;
end;


procedure TForm1.Timer1Timer(Sender: TObject);
begin
  Caption := 'Pathfinding Recast/Detour/Crowd ' + Format('%.1f', [1000 / fFrameTime]);
  if fSample <> nil then
    fSample.handleUpdate(0.05);
end;


procedure TForm1.UpdateModelViewProjection;
var
  dx, dy, dz: Single;
  eyeX, eyeY, eyeZ: Single;
begin
  {$POINTERMATH ON}
  glMatrixMode(GL_PROJECTION); //Change Matrix Mode to Projection
  glLoadIdentity;
  gluPerspective(45, Panel3.Width / Panel3.Height, 0.01, 1000);

  dx := 0; dy := 0; dz := 0;
  if fSample <> nil then
  begin
    dx := (fSample.getBoundsMin[0] + fSample.getBoundsMax[0]) / 2;
    dy := (fSample.getBoundsMin[1] + fSample.getBoundsMax[1]) / 2;
    dz := (fSample.getBoundsMin[2] + fSample.getBoundsMax[2]) / 2;
  end;

  eyeX := Sin(fRotateX / 180 * pi) * Cos(fRotateY / 180 * pi) * fDist;
  eyeY := Cos(fRotateX / 180 * pi) * Cos(fRotateY / 180 * pi) * fDist;
  eyeZ := Sin(fRotateY / 180 * pi) * fDist;

  glMatrixMode(GL_MODELVIEW); //Return to the modelview matrix
  glLoadIdentity;
  gluLookAt(eyeX + dx, eyeY + dy, eyeZ + dz, dx, dy, dz, 0, 0, 1);
  {$POINTERMATH OFF}
end;


end.
