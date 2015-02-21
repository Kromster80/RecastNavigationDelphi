unit RN_ValueHistory;
interface

type
  TValueHistory = class
  private
    const MAX_HISTORY = 256;
    var
    m_samples: array [0..MAX_HISTORY-1] of Single;
    m_hsamples: Integer;
  public
    constructor Create();
    destructor Destroy; override;

    procedure addSample(const val: Single);
    function getSampleCount(): Integer;
    function getSample(const i: Integer): Single;

    function getSampleMin(): Single;
    function getSampleMax(): Single;
    function getAverage(): Single;
  end;

  PGraphParams = ^TGraphParams;
  TGraphParams = record
    procedure setRect(ix, iy, iw, ih, ipad: Integer);
    procedure setValueRange(ivmin, ivmax: Single; indiv: Integer; const iunits: PShortInt);
  public
    x, y, w, h, pad: Integer;
    vmin, vmax: Single;
    ndiv: Integer;
    units: array [0..15] of ShortInt;
  end;

  procedure drawGraphBackground(const p: PGraphParams);

  procedure drawGraph(const p: PGraphParams; const graph: TValueHistory;
           idx: Integer; const &label: PShortInt; const col: Cardinal);


implementation


constructor TValueHistory.Create();
var i: Integer;
begin
  inherited;

  for i := 0 to MAX_HISTORY - 1 do
    m_samples[i] := 0;
end;

destructor TValueHistory.Destroy;
begin
  inherited;
end;

procedure TValueHistory.addSample(const val: Single);
begin
  m_hsamples := (m_hsamples+MAX_HISTORY-1) mod MAX_HISTORY;
  m_samples[m_hsamples] := val;
end;

function TValueHistory.getSampleCount(): Integer;
begin
  Result := MAX_HISTORY;
end;

function TValueHistory.getSample(const i: Integer): Single;
begin
  Result := m_samples[(m_hsamples+i) mod MAX_HISTORY];
end;

function TValueHistory.getSampleMin(): Single;
var val: Single; i: Integer;
begin
  val := m_samples[0];
  for i := 1 to MAX_HISTORY - 1 do
    if (m_samples[i] < val) then
      val := m_samples[i];
  Result := val;
end;

function TValueHistory.getSampleMax(): Single;
var val: Single; i: Integer;
begin
  val := m_samples[0];
  for i := 1 to MAX_HISTORY - 1 do
    if (m_samples[i] > val) then
      val := m_samples[i];
  Result := val;
end;

function TValueHistory.getAverage(): Single;
var val: Single; i: Integer;
begin
  val := 0;
  for i := 1 to MAX_HISTORY - 1 do
    val := val + m_samples[i];
  Result := val/MAX_HISTORY;
end;

procedure TGraphParams.setRect(ix, iy, iw, ih, ipad: Integer);
begin
  x := ix;
  y := iy;
  w := iw;
  h := ih;
  pad := ipad;
end;

procedure TGraphParams.setValueRange(ivmin, ivmax: Single; indiv: Integer; const iunits: PShortInt);
begin
  vmin := ivmin;
  vmax := ivmax;
  ndiv := indiv;
  //todo: Delphi: strcpy(units, iunits);
end;

procedure drawGraphBackground(const p: PGraphParams);
begin
{  // BG
  imguiDrawRoundedRect((float)p.x, (float)p.y, (float)p.w, (float)p.h, (float)p.pad, imguiRGBA(64,64,64,128));

  const float sy := (p.h-p.pad*2) / (p.vmax-p.vmin);
  const float oy := p.y+p.pad-p.vmin*sy;

  char text[64];

  // Divider Lines
  for (int i := 0; i <= p.ndiv; ++i)
  begin
    const float u := (float)i/(float)p.ndiv;
    const float v := p.vmin + (p.vmax-p.vmin)*u;
    snprintf(text, 64, "%.2f %s", v, p.units);
    const float fy := oy + v*sy;
    imguiDrawText(p.x + p.w - p.pad, (int)fy-4, IMGUI_ALIGN_RIGHT, text, imguiRGBA(0,0,0,255));
    imguiDrawLine((float)p.x + (float)p.pad, fy, (float)p.x + (float)p.w - (float)p.pad - 50, fy, 1.0f, imguiRGBA(0,0,0,64));
  end;}
end;

procedure drawGraph(const p: PGraphParams; const graph: TValueHistory;
           idx: Integer; const &label: PShortInt; const col: Cardinal);
begin
{  const float sx := (p.w - p.pad*2) / (float)graph.getSampleCount();
  const float sy := (p.h - p.pad*2) / (p.vmax - p.vmin);
  const float ox := (float)p.x + (float)p.pad;
  const float oy := (float)p.y + (float)p.pad - p.vmin*sy;

  // Values
  float px=0, py=0;
  for (int i := 0; i < graph.getSampleCount()-1; ++i)
  begin
    const float x := ox + i*sx;
    const float y := oy + graph.getSample(i)*sy;
    if (i > 0)
      imguiDrawLine(px,py, x,y, 2.0f, col);
    px := x;
    py := y;
  end;

  // Label
  const int size := 15;
  const int spacing := 10;
  int ix := p.x + p.w + 5;
  int iy := p.y + p.h - (idx+1)*(size+spacing);

  imguiDrawRoundedRect((float)ix, (float)iy, (float)size, (float)size, 2.0f, col);

  char text[64];
  snprintf(text, 64, "%.2f %s", graph.getAverage(), p.units);
  imguiDrawText(ix+size+5, iy+3, IMGUI_ALIGN_LEFT, label, imguiRGBA(255,255,255,192));
  imguiDrawText(ix+size+150, iy+3, IMGUI_ALIGN_RIGHT, text, imguiRGBA(255,255,255,128));}
end;

end.
