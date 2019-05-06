classdef BSTElement < handle
% Reference: Brian Moore, Data Structure, MATLAB File exchange
% https://www.mathworks.com/matlabcentral/fileexchange/45123-data-structures
% [One additional function key_value(this) is added]

% A class that implements pass by reference elements with numeric keys for
% instances of the BinarySearchTree class
%
% NOTE: This class is used internally by the BinarySearchTree class
%

    %
    % Public properties
    %
	properties (Access = public)
		key;            % key
        left = nan;     % left child
        right = nan;    % right child
        p = nan;        % parent
        value = [];     % miscellaneous data
    end
    
    %
    % Public methods
    %
	methods (Access = public)
        %
        % Constructor
        %
		function this = BSTElement(key,value)
			% Initialize key
            this.key = key;
            
            % Set value data, if specified
            if (nargin == 2)
                this.value = value;
            end
        end
        
        %
        % Element is nan if its key is nan
        %
        function bool = isnan(this)
            bool = isnan(this.key);
        end
        
        function [k,v] = key_value(this)
            k = this.key;
            v = this.value;
        end
        
        
    end
end
