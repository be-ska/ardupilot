local count = 0
local RELAY_NUM1 = 0
local LOITER_MODE = 5
local RTL_MODE = 6
local AUTO_MODE = 3
local LAND_MODE = 9
function relay_on()
    relay:on(RELAY_NUM1)
end
function relay_off()
    relay:off(RELAY_NUM1)
end
function led()
    if arming:is_armed() then
        -- if armed different flash pattern with different flight modes
        if vehicle:get_mode() == LOITER_MODE then
            if count == 0 or count == 2 then
                relay_off()
            else
                relay_on()
            end
        elseif vehicle:get_mode() == AUTO_MODE then
            if count == 0 or count == 2 or count == 4 then
                relay_off()
            else
                relay_on()
            end
        elseif vehicle:get_mode() == LAND_MODE or vehicle:get_mode() == RTL_MODE then
            if count < 3 then
                relay_off()
            else
                relay_on()
            end
        else
            if count == 0 then
                relay_off()
            else
                relay_on()
            end
        end
        count = count + 1
        if count > 9 then
            count = 0
    end
    else
        relay_off()
    end
    return led, 200
end
return led, 10
