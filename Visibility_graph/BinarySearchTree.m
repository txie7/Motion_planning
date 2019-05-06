classdef BinarySearchTree < handle
% Reference: Brian Moore, Data Structure, MATLAB File exchange
% https://www.mathworks.com/matlabcentral/fileexchange/45123-data-structures
%--------------------------------------------------------------------------
% Class:        BinarySearchTree < handle
%               
% Constructor:  T = BinarySearchTree();
%               
% Properties:   (none)
%               
% Methods:              T.Insert(key,value);
%                       T.Delete(x);
%               key   = T.Sort();
%               bool  = T.ContainsKey(key);
%               x     = T.Search(key);
%               x     = T.Minimum();
%               y     = T.NextSmallest(x);
%               x     = T.Maximum();
%               y     = T.NextLargest(x);
%               count = T.Count();
%               bool  = T.IsEmpty();
%                       T.Clear();
%               
% Description:  This class implements a binary search tree with numeric
%               keys and arbitrarily typed values
%               
% Author:       Brian Moore
%               brimoor@umich.edu
%               
% Date:         January 16, 2014
%--------------------------------------------------------------------------

    %
    % Private properties
    %
    properties (Access = public)
        k;                  % current number of elements
        root;               % tree root pointer
    end
    
    %
    % Public methods
    %
    methods (Access = public)
        %
        % Constructor
        %
        function this = BinarySearchTree()
            %----------------------- Constructor --------------------------
            % Syntax:       T = BinarySearchTree();
            %               
            % Description:  Creates an empty binary search tree
            %--------------------------------------------------------------
            
            % Start with an empty tree
            this.Clear();
        end
        
        %
        % Create element from key-value pair and insert into tree
        %
        function Insert(this,key,value)
            %------------------------- Insert -----------------------------
            % Syntax:       T.Insert(key,value);
            %               
            % Inputs:       key is a numeric key
            %               
            %               value is an arbitrary object
            %               
            % Description:  Inserts the given key-value pair into T
            %--------------------------------------------------------------
            
            % Create element from data
            if (nargin == 2)
                x = BSTElement(key);
            else
                x = BSTElement(key,value);
            end
            
            % Insert z into tree
            y = nan;
            z = this.root;
            while ~isnan(z)
               y = z;
               if (x.key < z.key)
                   z = z.left;
               else
                   z = z.right;
               end
            end
            x.p = y;
            if isnan(y)
                this.root = x;
            elseif (x.key < y.key)
                y.left = x;
            else
                y.right = x;
            end
            
            % Update length
            this.k = this.k + 1;
        end
        
        %
        % Delete element from tree
        %
        function Delete(this,x)
            %------------------------- Delete -----------------------------
            % Syntax:       T.Delete(x);
            %               
            % Inputs:       x is a node in T (i.e., an object of type
            %               BSTElement) presumably extracted via a
            %               prior operation on T
            %               
            % Description:  Deletes the node x from T
            %--------------------------------------------------------------
            
            % Delete element
            if ~isnan(x)
                if isnan(x.left)
                    this.Transplant(x,x.right);
                elseif isnan(x.right)
                    this.Transplant(x,x.left);
                else
                    y = BinarySearchTree.TreeMinimum(x.right);
                    if (y.p ~= x)
                        this.Transplant(y,y.right);
                        y.right = x.right;
                        y.right.p = y;
                    end
                    this.Transplant(x,y);
                    y.left = x.left;
                    y.left.p = y;
                end
                
                % Update length
                this.k = this.k - 1;
            end
        end
        
        %
        % Return array of sorted keys
        %
        function keys = Sort(this)
            %-------------------------- Sort ------------------------------
            % Syntax:       keys = T.Sort();
            %               
            % Outputs:      keys is a column vector containing the sorted
            %               (ascending order) keys contained in T
            %               
            % Description:  Returns the sorted keys in T
            %--------------------------------------------------------------
            
            keys = BSTElement(1,zeros(this.k,1));
            BinarySearchTree.InOrderTreeWalk(this.root,keys);
            keys = keys.value;
        end
        
        %
        % Determine if tree contains element with given key
        %
        function bool = ContainsKey(this,key)
            %---------------------- ContainsKey ---------------------------
            % Syntax:       bool = T.ContainsKeys(key);
            %               
            % Inputs:       key is a numeric key
            %               
            % Outputs:      bool = {true,false}
            %               
            % Description:  Determines if T contains an element with the
            %               given key
            %--------------------------------------------------------------
            
            bool = ~isnan(this.Search(key));
        end
        
        %
        % Return element with given key
        %
        function x = Search(this,key)
            %------------------------- Search -----------------------------
            % Syntax:       x = T.Search(key);
            %               
            % Inputs:       key is a numeric key
            %               
            % Outputs:      x is a node from T (i.e., an object of class
            %               BSTElement) with the given key, if it exists in
            %               T, and NaN otherwise 
            %               
            % Description:  Returns node from T with the given key
            %--------------------------------------------------------------
            
            x = this.root;
            while (~isnan(x) && (key ~= x.key))
                if (key < x.key)
                    x = x.left;
                else
                    x = x.right;
                end
            end
        end
        
        %
        % Return pointer to element with smallest key
        %
        function x = Minimum(this)
            %------------------------- Minimum ----------------------------
            % Syntax:       x = T.Minimum();
            %               
            % Outputs:      x is the node from T (i.e., an object of class
            %               BSTElement) with the smallest key
            %               
            % Description:  Returns node from T with the smallest key
            %--------------------------------------------------------------
            
            x = BinarySearchTree.TreeMinimum(this.root);
        end
        
        %
        % Return element with next smallest key (predecessor) of x
        %
        function y = NextSmallest(this,x) %#ok
            %---------------------- NextSmallest --------------------------
            % Syntax:       y = T.NextSmallest(x);
            %               
            % Inputs:       x is a node from T (i.e., an object of class
            %               BSTElement)
            %               
            % Outputs:      y is the node from T (i.e., an object of class
            %               BSTElement) with the next smallest key than x
            %               
            % Description:  Returns node from T with the next smallest key
            %               than the input node
            %--------------------------------------------------------------
            
            if ~isnan(x)
                if ~isnan(x.left)
                    y = BinarySearchTree.TreeMaximum(x.left);
                else
                    y = x.p;
                    while (~isnan(y) && (x == y.left))
                        x = y;
                        y = y.p;
                    end
                end
            else
                y = nan;
            end
        end
        
        %
        % Return pointer to element with largest key
        %
        function x = Maximum(this)
            %------------------------- Maximum ----------------------------
            % Syntax:       x = T.Maximum();
            %               
            % Outputs:      x is the node from T (i.e., an object of class
            %               BSTElement) with the largest key
            %               
            % Description:  Returns node from T with the largest key
            %--------------------------------------------------------------
            
            x = BinarySearchTree.TreeMaximum(this.root);
        end
        
        %
        % Return element with next largest key (successor) of x
        %
        function y = NextLargest(this,x) %#ok
            %---------------------- NextLargest --------------------------
            % Syntax:       y = T.NextLargest(x);
            %               
            % Inputs:       x is a node from T (i.e., an object of class
            %               BSTElement)
            %               
            % Outputs:      y is the node from T (i.e., an object of class
            %               BSTElement) with the next largest key than x
            %               
            % Description:  Returns node from T with the next largest key
            %               than the input node
            %--------------------------------------------------------------
            
            if ~isnan(x)
                if ~isnan(x.right)
                    y = BinarySearchTree.TreeMinimum(x.right);
                else
                    y = x.p;
                    while (~isnan(y) && (x == y.right))
                        x = y;
                        y = y.p;
                    end
                end
            else
                y = nan;
            end
        end
        
        %
        % Return number of elements in tree
        %
        function count = Count(this)
            %-------------------------- Count -----------------------------
            % Syntax:       count = T.Count();
            %               
            % Outputs:      count is the number of nodes in T
            %               
            % Description:  Returns number of elements in T
            %--------------------------------------------------------------
            
            count = this.k;
        end
        
        %
        % Check for empty tree
        %
        function bool = IsEmpty(this)
            %------------------------ IsEmpty -----------------------------
            % Syntax:       bool = T.IsEmpty();
            %               
            % Outputs:      bool = {true,false}
            %               
            % Description:  Determines if T is empty (i.e., contains zero
            %               elements)
            %--------------------------------------------------------------
            
            if (this.k == 0)
                bool = true;
            else
                bool = false;
            end
        end
        
        %
        % Clear the tree
        %
        function Clear(this)
            %------------------------- Clear ------------------------------
            % Syntax:       T.Clear();
            %               
            % Description:  Removes all elements from T
            %--------------------------------------------------------------
            
            this.k = 0;         % reset length counter
            this.root = nan;    % reset root pointer
        end
    end
    
    %
    % Private methods
    %
    methods (Access = private)
        %
        % Replaces the subtree rooted at element u with the subtree
        % rooted at element v
        %
        function Transplant(this,u,v)
            if isnan(u.p)
                this.root = v;
            elseif (u == u.p.left)
                u.p.left = v;
            else
                u.p.right = v;
            end
            if ~isnan(v)
                v.p = u.p;
            end
        end
    end
    
    %
    % Private static methods
    %
    methods (Access = private, Static = true)
        %
        % In-order tree walk from given element
        %
        function InOrderTreeWalk(x,keys)
            if ~isnan(x)
                BinarySearchTree.InOrderTreeWalk(x.left,keys);
                keys.value(keys.key) = x.key;
                keys.key = keys.key + 1;
                BinarySearchTree.InOrderTreeWalk(x.right,keys);
            end
        end
        
        %
        % Return pointer to minimum of subtree rooted at x
        %
        function x = TreeMinimum(x)
            if ~isnan(x)
                while ~isnan(x.left)
                    x = x.left;
                end
            end
        end
        
        %
        % Return pointer to maximum of subtree rooted at x
        %
        function x = TreeMaximum(x)
            if ~isnan(x)
                while ~isnan(x.right)
                    x = x.right;
                end
            end
        end
    end
end
