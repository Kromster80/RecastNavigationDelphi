object FrameCrowdTool: TFrameCrowdTool
  Left = 0
  Top = 0
  Width = 320
  Height = 240
  TabOrder = 0
  object Label1: TLabel
    Left = 8
    Top = 96
    Width = 37
    Height = 13
    Caption = 'Options'
  end
  object rgCrowdToolMode: TRadioGroup
    Left = 8
    Top = 8
    Width = 185
    Height = 81
    Caption = 'rgCrowdToolMode'
    Items.Strings = (
      'Create Agents'
      'Move Target'
      'Select Agent'
      'Toggle Polys')
    TabOrder = 0
  end
  object cbOptimizeVisibility: TCheckBox
    Left = 8
    Top = 112
    Width = 105
    Height = 17
    Caption = 'Optimize Visibility'
    TabOrder = 1
  end
  object cbOptimizeTopology: TCheckBox
    Left = 8
    Top = 128
    Width = 105
    Height = 17
    Caption = 'Optimize Topology'
    TabOrder = 2
  end
  object cbAnticipateTurns: TCheckBox
    Left = 8
    Top = 144
    Width = 97
    Height = 17
    Caption = 'Anticipate Turns'
    TabOrder = 3
  end
  object cbObstacleAvoidance: TCheckBox
    Left = 8
    Top = 160
    Width = 113
    Height = 17
    Caption = 'Obstacle Avoidance'
    TabOrder = 4
  end
end
