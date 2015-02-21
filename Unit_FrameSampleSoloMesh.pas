unit Unit_FrameSampleSoloMesh;
interface
uses
  Vcl.Forms, Vcl.Controls, Vcl.StdCtrls, System.Classes, Vcl.ExtCtrls, Vcl.Samples.Spin;

type
  TFrameSampleSoloMesh = class(TFrame)
    Label13: TLabel;
    Label12: TLabel;
    Label11: TLabel;
    Label10: TLabel;
    Label9: TLabel;
    Label8: TLabel;
    Label7: TLabel;
    Label6: TLabel;
    Label5: TLabel;
    Label4: TLabel;
    Label2: TLabel;
    seCellSize: TSpinEdit;
    seMaxSampleError: TSpinEdit;
    seSampleDistance: TSpinEdit;
    seVertsPerPoly: TSpinEdit;
    seMaxEdgeError: TSpinEdit;
    seMaxEdgeLength: TSpinEdit;
    rgPartitioning: TRadioGroup;
    seMergedRegionSize: TSpinEdit;
    seMinRegionSize: TSpinEdit;
    seMaxSlope: TSpinEdit;
    seMaxClimb: TSpinEdit;
    seAgentRadius: TSpinEdit;
    seAgentHeight: TSpinEdit;
    seCellHeight: TSpinEdit;
    chkKeepIntermediateResults: TCheckBox;
    rgDrawMode: TRadioGroup;
    Label1: TLabel;
    Label3: TLabel;
  end;

implementation

{$R *.dfm}


end.
