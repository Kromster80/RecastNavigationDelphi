unit Unit_FrameTesterTool;
interface
uses
  Vcl.Forms, Vcl.Controls, Vcl.StdCtrls, System.Classes, Vcl.ExtCtrls;

type
  TFrameTesterTool = class(TFrame)
    rgToolMode: TRadioGroup;
    rgVerticesAtCrossings: TRadioGroup;
    btnSetRandomStart: TButton;
    btnSetRandomEnd: TButton;
    btnMakeRandomPoints: TButton;
    btnMakeRandomPointsAround: TButton;
    cbInclWalk: TCheckBox;
    cbInclSwim: TCheckBox;
    cbInclDoor: TCheckBox;
    cbInclJump: TCheckBox;
    Label1: TLabel;
    cbExclWalk: TCheckBox;
    cbExclSwim: TCheckBox;
    cbExclDoor: TCheckBox;
    cbExclJump: TCheckBox;
    Label2: TLabel;
  end;

implementation

{$R *.dfm}


end.
