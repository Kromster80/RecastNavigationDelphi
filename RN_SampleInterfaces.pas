//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//
unit RN_SampleInterfaces;
interface
uses Math, OpenGL, StrUtils, SysUtils,
  RN_Helper, RN_Recast, RN_DebugDraw;

// These are example implementations of various interfaces used in Recast and Detour.

type
/// Recast build context.
  TBuildContext = class(TrcContext)
  private
    m_startTime: array [TrcTimerLabel] of Int64;
    m_accTime: array [TrcTimerLabel] of Integer;

    const MAX_MESSAGES = 1000;
    var
    m_messages: array [0..MAX_MESSAGES-1] of string;
    m_messageCount: Integer;
    //const TEXT_POOL_SIZE = 8000;
    //var
    //m_textPool: array [0..TEXT_POOL_SIZE-1] of Char;
    //m_textPoolSize: Integer;

  protected
    /// Virtual functions for custom implementations.
    ///@
    procedure doResetLog(); override;
    procedure doLog(category: TrcLogCategory; msg: string); override;
    procedure doResetTimers(); override;
    procedure doStartTimer(const &label: TrcTimerLabel); override;
    procedure doStopTimer(const &label: TrcTimerLabel); override;
    function doGetAccumulatedTime(const &label: TrcTimerLabel): Integer; override;
    ///@
  public
    /// Dumps the log to stdout.
    procedure dumpLog(format: string);
    /// Returns number of log messages.
    function getLogCount: Integer;
    /// Returns log message text.
    function getLogText(const i: Integer): string;
  end;

  /// OpenGL debug draw implementation.
  TDebugDrawGL = class(TduDebugDraw)
  public
    procedure depthMask(state: Boolean); override;
    procedure texture(state: Boolean); override;
    procedure &begin(prim: TduDebugDrawPrimitives; size: Single = 1.0); override;
    procedure vertex(const pos: PSingle; color: Cardinal);  override;
    procedure vertex(const x,y,z: Single; color: Cardinal);  override;
    procedure vertex(const pos: PSingle; color: Cardinal; uv: PSingle);  override;
    procedure vertex(const x,y,z: Single; color: Cardinal; u,v: Single);  override;
    procedure &end(); override;
  end;

  {/// stdio file implementation.
  TFileIO = class(TduFileIO)
  private
    FILE* m_fp;
    int m_mode;
  public
    FileIO();
    virtual ~FileIO();
    bool openForWrite(const char* path);
    bool openForRead(const char* path);
    virtual bool isWriting() const;
    virtual bool isReading() const;
    virtual bool write(const void* ptr, const size_t size);
    virtual bool read(void* ptr, const size_t size);
  end;}

implementation
uses RN_PerfTimer;
////////////////////////////////////////////////////////////////////////////////////////////////////

// Virtual functions for custom implementations.
procedure TBuildContext.doResetLog();
begin
  m_messageCount := 0;
  //m_textPoolSize := 0;
end;

procedure TBuildContext.doLog(category: TrcLogCategory; msg: string);
const Cat: array [TrcLogCategory] of String = ('','W','E');
begin
  if (msg = '') then Exit;
  if (m_messageCount >= MAX_MESSAGES) then
    Exit;

  // Store message
  m_messages[m_messageCount] := cat[category] + ' ' + msg;
  Inc(m_messageCount);
end;

procedure TBuildContext.doResetTimers();
var i: TrcTimerLabel;
begin
  for i := Low(TrcTimerLabel) to High(TrcTimerLabel) do
    m_accTime[i] := -1;
end;

procedure TBuildContext.doStartTimer(const &label: TrcTimerLabel);
begin
  m_startTime[&label] := getPerfTime();
end;

procedure TBuildContext.doStopTimer(const &label: TrcTimerLabel);
var endTime: Int64; deltaTime: Integer;
begin
  endTime := getPerfTime();
  deltaTime := Integer(endTime - m_startTime[&label]);
  if (m_accTime[&label] = -1) then
    m_accTime[&label] := deltaTime
  else
    Inc(m_accTime[&label], deltaTime);
end;

function TBuildContext.doGetAccumulatedTime(const &label: TrcTimerLabel): Integer;
begin
  Result := m_accTime[&label];
end;

procedure TBuildContext.dumpLog(format: string);
begin
  {// Print header.
  va_list ap;
  va_start(ap, format);
  vprintf(format, ap);
  va_end(ap);
  printf("\n");

  // Print messages
  const int TAB_STOPS[4] := begin 28, 36, 44, 52 end;;
  for (int i := 0; i < m_messageCount; ++i)
  begin
    const char* msg := m_messages[i]+1;
    int n := 0;
    while ( *msg)
    begin
      if ( *msg == '\t')
      begin
        int count := 1;
        for (int j := 0; j < 4; ++j)
        begin
          if (n < TAB_STOPS[j])
          begin
            count := TAB_STOPS[j] - n;
            break;
          end;
        end;
        while (--count)
        begin
          putchar(' ');
          n++;
        end;
      end;
      else
      begin
        putchar( *msg);
        n++;
      end;
      msg++;
    end;
    putchar('\n');
  end;}
end;

function TBuildContext.getLogCount: Integer;
begin
  Result := m_messageCount;
end;

function TBuildContext.getLogText(const i: Integer): string;
begin
  Result := m_messages[i];
end;

////////////////////////////////////////////////////////////////////////////////////////////////////

{class GLCheckerTexture
begin
  unsigned int m_texId;
public:
  GLCheckerTexture() : m_texId(0)
  begin
  end;

  ~GLCheckerTexture()
  begin
    if (m_texId != 0)
      glDeleteTextures(1, &m_texId);
  end;
  void bind()
  begin
    if (m_texId == 0)
    begin
      // Create checker pattern.
      const unsigned int col0 := duRGBA(215,215,215,255);
      const unsigned int col1 := duRGBA(255,255,255,255);
      static const int TSIZE := 64;
      unsigned int data[TSIZE*TSIZE];

      glGenTextures(1, &m_texId);
      glBindTexture(GL_TEXTURE_2D, m_texId);

      int level := 0;
      int size := TSIZE;
      while (size > 0)
      begin
        for (int y := 0; y < size; ++y)
          for (int x := 0; x < size; ++x)
            data[x+y*size] := (x==0 || y==0) ? col0 : col1;
        glTexImage2D(GL_TEXTURE_2D, level, GL_RGBA, size,size, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        size /= 2;
        level++;
      end;

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    end;
    else
    begin
      glBindTexture(GL_TEXTURE_2D, m_texId);
    end;
  end;
end;;
GLCheckerTexture g_tex;}


procedure TDebugDrawGL.depthMask(state: Boolean);
begin
  if state then
    glDepthMask(GL_TRUE)
  else
    glDepthMask(GL_FALSE);
end;

procedure TDebugDrawGL.texture(state: Boolean);
begin
  if (state) then
  begin
    glEnable(GL_TEXTURE_2D);
    //todo: g_tex.bind();
  end
  else
  begin
    glDisable(GL_TEXTURE_2D);
  end;
end;

procedure TDebugDrawGL.&begin(prim: TduDebugDrawPrimitives; size: Single);
begin
  case prim of
    DU_DRAW_POINTS: begin
      glPointSize(size);
      glBegin(GL_POINTS);
    end;
    DU_DRAW_LINES: begin
      glLineWidth(size);
      glBegin(GL_LINES);
    end;
    DU_DRAW_TRIS:
      glBegin(GL_TRIANGLES);
    DU_DRAW_QUADS:
      glBegin(GL_QUADS);
  end;
end;

procedure TDebugDrawGL.vertex(const pos: PSingle; color: Cardinal);
begin
  glColor4ubv(@color);
  glVertex3fv(Pointer(pos));
end;

procedure TDebugDrawGL.vertex(const x,y,z: Single; color: Cardinal);
begin
  glColor4ubv(@color);
  glVertex3f(x,y,z);
end;

procedure TDebugDrawGL.vertex(const pos: PSingle; color: Cardinal; uv: PSingle);
begin
  glColor4ubv(@color);
  glTexCoord2fv(Pointer(uv));
  glVertex3fv(Pointer(pos));
end;

procedure TDebugDrawGL.vertex(const x,y,z: Single; color: Cardinal; u,v: Single);
begin
  glColor4ubv(@color);
  glTexCoord2f(u,v);
  glVertex3f(x,y,z);
end;

procedure TDebugDrawGL.&end();
begin
  glEnd();
  glLineWidth(1.0);
  glPointSize(1.0);
end;

{////////////////////////////////////////////////////////////////////////////////////////////////////

FileIO::FileIO() :
  m_fp(0),
  m_mode(-1)
begin
end;

FileIO::~FileIO()
begin
  if (m_fp) fclose(m_fp);
end;

bool FileIO::openForWrite(const char* path)
begin
  if (m_fp) return false;
  m_fp := fopen(path, "wb");
  if (!m_fp) return false;
  m_mode := 1;
  return true;
end;

bool FileIO::openForRead(const char* path)
begin
  if (m_fp) return false;
  m_fp := fopen(path, "rb");
  if (!m_fp) return false;
  m_mode := 2;
  return true;
end;

bool FileIO::isWriting() const
begin
  return m_mode == 1;
end;

bool FileIO::isReading() const
begin
  return m_mode == 2;
end;

bool FileIO::write(const void* ptr, const size_t size)
begin
  if (!m_fp || m_mode != 1) return false;
  fwrite(ptr, size, 1, m_fp);
  return true;
end;

bool FileIO::read(void* ptr, const size_t size)
begin
  if (!m_fp || m_mode != 2) return false;
  size_t readLen := fread(ptr, size, 1, m_fp);
  return readLen == 1;
end;}

end.


