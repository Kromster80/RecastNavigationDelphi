unit Unit_FramePruneTool;
interface
uses
  Vcl.Forms, Vcl.Controls, Vcl.StdCtrls, System.Classes;

type
  TFramePruneTool = class(TFrame)
    btnClearSelection: TButton;
    btnPruneUnselected: TButton;
  end;

implementation

{$R *.dfm}


end.
