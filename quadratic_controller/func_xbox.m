%% Function to Detect Xbox Input
function [] = func_xbox(button,params)

if button(1)
    if button(3)
        params.controls.pos_v = params.controls.pos_v + [params.keyboard.inc_pos_v,0,0]';
    end
    
    if button(4)
        params.controls.pos_v = params.controls.pos_v + [0,params.keyboard.inc_pos_v,0]';
    end
    
    if button(5)
        params.controls.pos_v = params.controls.pos_v + [0,0,params.keyboard.inc_pos_v]';
    end
end

if button(2)
    if button(3)
        params.controls.pos_v = params.controls.pos_v + [-params.keyboard.inc_pos_v,0,0]';
    end
    
    if button(4)
        params.controls.pos_v = params.controls.pos_v + [0,-params.keyboard.inc_pos_v,0]';
    end
    
    if button(5)
        params.controls.pos_v = params.controls.pos_v + [0,0,-params.keyboard.inc_pos_v]';
    end
end
end