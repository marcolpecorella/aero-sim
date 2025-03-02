import {Component, Input} from '@angular/core';
import {MatSidenav, MatSidenavContainer, MatSidenavModule} from '@angular/material/sidenav';
import {MatListItem, MatNavList} from '@angular/material/list';

@Component({
  selector: 'app-sidebar',
  imports: [
    MatSidenavContainer,
    MatNavList,
    MatListItem,
    MatSidenav,
    MatSidenavModule,
  ],
  templateUrl: './sidebar.component.html',
  styleUrl: './sidebar.component.css'
})
export class SidebarComponent {
  @Input() isSidebarOpen: boolean = false;

}
