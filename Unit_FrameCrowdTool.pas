unit Unit_FrameCrowdTool;
interface
uses
  Vcl.Forms, Vcl.Controls, Vcl.StdCtrls, System.Classes, Vcl.ExtCtrls;

type
  TFrameCrowdTool = class(TFrame)
    rgCrowdToolMode: TRadioGroup;
    cbOptimizeVisibility: TCheckBox;
    cbOptimizeTopology: TCheckBox;
    cbAnticipateTurns: TCheckBox;
    cbObstacleAvoidance: TCheckBox;
    Label1: TLabel;
  end;

implementation

{$R *.dfm}


end.
